#include "pose_graph.h"
#include "voxblox_mapper.h"
#include <algorithm>
#include <cmath>
extern float RESOLUTION;
extern int PCL_FILTER_MIN_DENSITY;
extern int USE_DEPTH_TO_MAP_POSE_GRAPH;
extern double DEPTH_MAP_WEIGHT;
extern double DEPTH_MAP_HUBER;
extern int DEPTH_MAP_MIN_EDGES;
extern int DEPTH_MAP_MAX_EDGES_PER_FRAME;
extern int DEPTH_MAP_NEIGHBOR_COUNT;
extern double DEPTH_MAP_MAX_NEIGHBOR_DIST;
extern double DEPTH_MAP_PLANE_MAX_DIST;
extern double DEPTH_MAP_MIN_SCALE;
extern int DEPTH_MAP_POSE_GRAPH_TARGET_KEYFRAMES;
extern int DEPTH_MAP_POSE_GRAPH_OPT_INTERVAL;
extern Eigen::Matrix<double, 3, 1> ti_d;
extern Eigen::Matrix<double, 3, 3> qi_d;

namespace
{
struct PoseGraphDepthMapEdge
{
    int target_local_index;
    Vector3d point_c;
    Vector4d plane;
    double scale;
    double abs_distance;
};

pcl::PointXYZRGB makeColorPoint(const Vector3d &point_w, const cv::Vec3b &bgr)
{
    pcl::PointXYZRGB color_point;
    color_point.x = point_w(0);
    color_point.y = point_w(1);
    color_point.z = point_w(2);
    color_point.r = bgr[2];
    color_point.g = bgr[1];
    color_point.b = bgr[0];
    return color_point;
}

bool fitPlane(const std::vector<int> &indices,
              const pcl::PointCloud<pcl::PointXYZ>::Ptr &map_cloud,
              Vector4d &plane)
{
    Vector3d centroid = Vector3d::Zero();
    for (int idx : indices)
        centroid += Vector3d(map_cloud->points[idx].x, map_cloud->points[idx].y, map_cloud->points[idx].z);
    centroid /= static_cast<double>(indices.size());

    Matrix3d covariance = Matrix3d::Zero();
    for (int idx : indices)
    {
        Vector3d point(map_cloud->points[idx].x, map_cloud->points[idx].y, map_cloud->points[idx].z);
        Vector3d delta = point - centroid;
        covariance += delta * delta.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success)
        return false;

    Vector3d normal = solver.eigenvectors().col(0).normalized();
    plane.head<3>() = normal;
    plane(3) = -normal.dot(centroid);
    return true;
}

Vector3d transformDepthPoint(const Vector3d &point_c, const Matrix3d &R_w_i, const Vector3d &P_w_i)
{
    return R_w_i * (qi_d * point_c + ti_d) + P_w_i;
}

std::vector<PoseGraphDepthMapEdge> buildPoseGraphDepthMapEdges(const std::vector<KeyFrame*> &keyframes,
                                                               const Quaterniond *q_array,
                                                               double (*t_array)[3])
{
    std::vector<PoseGraphDepthMapEdge> edges;
    if (!USE_DEPTH_TO_MAP_POSE_GRAPH || keyframes.size() < 2)
        return edges;

    const int target_count = std::min<int>(std::max(1, DEPTH_MAP_POSE_GRAPH_TARGET_KEYFRAMES),
                                           static_cast<int>(keyframes.size()) - 1);
    const int max_edges_per_target = std::max(1, DEPTH_MAP_MAX_EDGES_PER_FRAME / target_count);
    const int first_target = static_cast<int>(keyframes.size()) - target_count;
    const int neighbor_count = std::max(3, DEPTH_MAP_NEIGHBOR_COUNT);
    const double max_neighbor_sq_dist = DEPTH_MAP_MAX_NEIGHBOR_DIST * DEPTH_MAP_MAX_NEIGHBOR_DIST;
    if (first_target <= 0)
        return edges;
    for (int target = first_target; target < static_cast<int>(keyframes.size()); ++target)
    {
        const std::vector<cv::Point3f> &target_points = keyframes[target]->point_3d_depth_raw;
        if (target_points.empty())
            continue;

        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (int frame = 0; frame < target; frame++)
        {
            if (keyframes[frame]->sequence != keyframes[target]->sequence)
                continue;

            Matrix3d R_w_i = q_array[frame].toRotationMatrix();
            Vector3d P_w_i(t_array[frame][0], t_array[frame][1], t_array[frame][2]);
            for (const cv::Point3f &point_cv : keyframes[frame]->point_3d_depth_raw)
            {
                Vector3d point_c(point_cv.x, point_cv.y, point_cv.z);
                Vector3d point_w = transformDepthPoint(point_c, R_w_i, P_w_i);
                map_cloud->push_back(pcl::PointXYZ(point_w.x(), point_w.y(), point_w.z()));
            }
        }
        if (static_cast<int>(map_cloud->size()) < DEPTH_MAP_MIN_EDGES)
            continue;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(map_cloud);
        Matrix3d target_R_w_i = q_array[target].toRotationMatrix();
        Vector3d target_P_w_i(t_array[target][0], t_array[target][1], t_array[target][2]);

        std::vector<PoseGraphDepthMapEdge> frame_edges;
        frame_edges.reserve(std::min<int>(target_points.size(), max_edges_per_target));
        int stride = std::max(1, static_cast<int>(target_points.size()) / max_edges_per_target);
        std::vector<int> indices(neighbor_count);
        std::vector<float> sq_distances(neighbor_count);
        for (int idx = 0; idx < static_cast<int>(target_points.size()) && static_cast<int>(frame_edges.size()) < max_edges_per_target; idx += stride)
        {
            Vector3d point_c(target_points[idx].x, target_points[idx].y, target_points[idx].z);
            Vector3d point_w = transformDepthPoint(point_c, target_R_w_i, target_P_w_i);
            pcl::PointXYZ query(point_w.x(), point_w.y(), point_w.z());
            if (kdtree.nearestKSearch(query, neighbor_count, indices, sq_distances) != neighbor_count)
                continue;
            if (sq_distances.back() > max_neighbor_sq_dist)
                continue;

            Vector4d plane;
            if (!fitPlane(indices, map_cloud, plane))
                continue;

            bool valid_plane = true;
            for (int nearest_idx : indices)
            {
                const pcl::PointXYZ &nearest = map_cloud->points[nearest_idx];
                double plane_distance = plane.head<3>().dot(Vector3d(nearest.x, nearest.y, nearest.z)) + plane(3);
                if (std::abs(plane_distance) > DEPTH_MAP_PLANE_MAX_DIST)
                {
                    valid_plane = false;
                    break;
                }
            }
            if (!valid_plane)
                continue;

            double distance = plane.head<3>().dot(point_w) + plane(3);
            double range2 = point_c.squaredNorm();
            if (range2 < 1e-6)
                continue;
            double scale = 1.0 - 0.9 * std::abs(distance) / std::sqrt(std::sqrt(range2));
            if (scale <= DEPTH_MAP_MIN_SCALE)
                continue;

            PoseGraphDepthMapEdge edge;
            edge.target_local_index = target;
            edge.point_c = point_c;
            edge.plane = plane;
            edge.scale = scale;
            edge.abs_distance = std::abs(distance);
            frame_edges.push_back(edge);
        }

        if (static_cast<int>(frame_edges.size()) >= DEPTH_MAP_MIN_EDGES)
            edges.insert(edges.end(), frame_edges.begin(), frame_edges.end());
    }

    return edges;
}
}

PoseGraph::PoseGraph()
{
    posegraph_visualization = new CameraPoseVisualization(1.0, 0.0, 1.0, 1.0);
    posegraph_visualization->setScale(0.1);
    posegraph_visualization->setLineWidth(0.01);
	t_optimization = std::thread(&PoseGraph::optimize4DoF, this);
    earliest_loop_index = -1;
    t_drift = Eigen::Vector3d(0, 0, 0);
    yaw_drift = 0;
    r_drift = Eigen::Matrix3d::Identity();
    w_t_vio = Eigen::Vector3d(0, 0, 0);
    w_r_vio = Eigen::Matrix3d::Identity();
    global_index = 0;
    sequence_cnt = 0;
    sequence_loop.push_back(0);
    base_sequence = 1;

}

PoseGraph::~PoseGraph()
{
	t_optimization.join();
}

void PoseGraph::registerPub(ros::NodeHandle &n)
{
    pub_pg_path = n.advertise<nav_msgs::Path>("pose_graph_path", 1000);
    pub_base_path = n.advertise<nav_msgs::Path>("base_path", 1000);
    pub_pose_graph = n.advertise<visualization_msgs::MarkerArray>("pose_graph", 1000);
    pub_octree = n.advertise<sensor_msgs::PointCloud2>("octree", 1000);
    for (int i = 1; i < 10; i++)
        pub_path[i] = n.advertise<nav_msgs::Path>("path_" + to_string(i), 1000);

    voxblox_mapper.reset(new VoxbloxMapper());
    voxblox_mapper->configure(n);
}

void PoseGraph::loadVocabulary(std::string voc_path)
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

void PoseGraph::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    //shift to base frame
    Vector3d vio_P_cur;
    Matrix3d vio_R_cur;
    // sequence = 1, sequence_cnt = 0 at init, sequence_cnt++
    // then sequence_cnt remains 1 (no new sequence in my case)
    if (sequence_cnt != cur_kf->sequence)
    {
        //run once
        sequence_cnt++;
        sequence_loop.push_back(0);
        w_t_vio = Eigen::Vector3d(0, 0, 0);
        w_r_vio = Eigen::Matrix3d::Identity();
        m_drift.lock();
        t_drift = Eigen::Vector3d(0, 0, 0);
        r_drift = Eigen::Matrix3d::Identity();
        m_drift.unlock();
    }
    cur_kf->getVioPose(vio_P_cur, vio_R_cur);
    vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
    vio_R_cur = w_r_vio *  vio_R_cur;
    cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
    cur_kf->index = global_index;
    global_index++;
	int loop_index = -1;
    bool queue_pose_optimization = false;
	// always true
    if (flag_detect_loop)
    {
        TicToc tmp_t;
        // get loop_index here
        loop_index = detectLoop(cur_kf, cur_kf->index);
    }
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }
	if (loop_index != -1)
	{
        //printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        KeyFrame* old_kf = getKeyFrame(loop_index);

        if (cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;

            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getVioPose(w_P_old, w_R_old);
            cur_kf->getVioPose(vio_P_cur, vio_R_cur);

            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = cur_kf->getLoopRelativeT();
            relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t;
            shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
            shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur;
            // shift vio pose of whole sequence to the world frame
            if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0)
            {
                w_r_vio = shift_r;
                w_t_vio = shift_t;
                vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                vio_R_cur = w_r_vio *  vio_R_cur;
                cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
                list<KeyFrame*>::iterator it = keyframelist.begin();
                for (; it != keyframelist.end(); it++)
                {
                    if((*it)->sequence == cur_kf->sequence)
                    {
                        Vector3d vio_P_cur;
                        Matrix3d vio_R_cur;
                        (*it)->getVioPose(vio_P_cur, vio_R_cur);
                        vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                        vio_R_cur = w_r_vio *  vio_R_cur;
                        (*it)->updateVioPose(vio_P_cur, vio_R_cur);
                    }
                }
                sequence_loop[cur_kf->sequence] = 1;
            }
            queue_pose_optimization = true;
        }
	}
	m_keyframelist.lock();
    Vector3d P;
    Matrix3d R;
    cur_kf->getVioPose(P, R);
    P = r_drift * P + t_drift;
    R = r_drift * R;
    cur_kf->updatePose(P, R);
    Quaterniond Q{R};
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();

    path[sequence_cnt].poses.push_back(pose_stamped);
    path[sequence_cnt].header = pose_stamped.header;

    sensor_msgs::PointCloud2 tmp_pcl;
    m_octree.lock();

    int pcl_count_temp = 0;
    for (unsigned int i = 0; i < cur_kf->point_3d_depth.size(); i++)
    {
        cv::Point3f pcl = cur_kf->point_3d_depth[i];
        Vector3d pts_i(pcl.x , pcl.y, pcl.z);
        Vector3d w_pts_i = R * (qi_d * pts_i + ti_d) + P;
        pcl::PointXYZ searchPoint;
        searchPoint.x = w_pts_i(0);
        searchPoint.y = w_pts_i(1);
        searchPoint.z = w_pts_i(2);
        if (octree->getVoxelDensityAtPoint(searchPoint) < 5)
        {
            cur_kf->point_3d_depth[pcl_count_temp] = pcl;
            if (i < cur_kf->point_3d_depth_color.size())
                cur_kf->point_3d_depth_color[pcl_count_temp] = cur_kf->point_3d_depth_color[i];
            octree->addPointToCloud(searchPoint, cloud);
            const cv::Vec3b bgr = i < cur_kf->point_3d_depth_color.size() ?
                cur_kf->point_3d_depth_color[i] : cv::Vec3b(128, 128, 128);
            const pcl::PointXYZRGB color_point = makeColorPoint(w_pts_i, bgr);
            color_cloud->push_back(color_point);
            save_cloud->push_back(color_point);
            ++pcl_count_temp;
        }
    }
    cur_kf->point_3d_depth.resize(pcl_count_temp);
    cur_kf->point_3d_depth_color.resize(pcl_count_temp);
    pcl::toROSMsg(*color_cloud, tmp_pcl);
    m_octree.unlock();

    if (voxblox_mapper && voxblox_mapper->enabled())
    {
        std::lock_guard<std::mutex> lock(m_voxblox);
        voxblox_mapper->integrateKeyFrame(cur_kf->point_3d_depth_raw,
                                          cur_kf->point_3d_depth_color_raw,
                                          R, P, qi_d, ti_d,
                                          ros::Time(cur_kf->time_stamp));
    }

    // not used
    if (SAVE_LOOP_PATH)
    {
        ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
        loop_path_file.setf(ios::fixed, ios::floatfield);
        loop_path_file.precision(0);
        loop_path_file << cur_kf->time_stamp * 1e9 << ",";
        loop_path_file.precision(5);
        loop_path_file  << P.x() << ","
              << P.y() << ","
              << P.z() << ","
              << Q.w() << ","
              << Q.x() << ","
              << Q.y() << ","
              << Q.z() << ","
              << endl;
        loop_path_file.close();
    }
    // not used
    //draw local connection
    if (SHOW_S_EDGE)
    {
        list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
        for (int i = 0; i < 4; i++)
        {
            if (rit == keyframelist.rend())
                break;
            Vector3d conncected_P;
            Matrix3d connected_R;
            if((*rit)->sequence == cur_kf->sequence)
            {
                (*rit)->getPose(conncected_P, connected_R);
                posegraph_visualization->add_edge(P, conncected_P);
            }
            rit++;
        }
    }
    // show connections between loop frames, not needed
    if (SHOW_L_EDGE)
    {
        if (cur_kf->has_loop)
        {
            //printf("has loop \n");
            KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
            Vector3d connected_P,P0;
            Matrix3d connected_R,R0;
            connected_KF->getPose(connected_P, connected_R);
            //cur_kf->getVioPose(P0, R0);
            cur_kf->getPose(P0, R0);
            if(cur_kf->sequence > 0)
            {
                //printf("add loop into visual \n");
                posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
            }

        }
    }
    //posegraph_visualization->add_pose(P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), Q);
    tmp_pcl.header = pose_stamped.header;
    // add frame to key frame list
	keyframelist.push_back(cur_kf);
    publish();
    pub_octree.publish(tmp_pcl);
	m_keyframelist.unlock();

    if (USE_DEPTH_TO_MAP_POSE_GRAPH && DEPTH_MAP_POSE_GRAPH_OPT_INTERVAL > 0 &&
        cur_kf->index > 0 && cur_kf->index % DEPTH_MAP_POSE_GRAPH_OPT_INTERVAL == 0)
    {
        queue_pose_optimization = true;
    }
    if (queue_pose_optimization)
    {
        m_optimize_buf.lock();
        optimize_buf.push(cur_kf->index);
        m_optimize_buf.unlock();
    }

}

void PoseGraph::setVoxbloxOutputDirectory(const std::string &output_dir)
{
    std::lock_guard<std::mutex> lock(m_voxblox);
    if (voxblox_mapper)
        voxblox_mapper->setOutputDirectory(output_dir);
}

void PoseGraph::saveVoxbloxMap()
{
    std::lock_guard<std::mutex> lock(m_voxblox);
    if (voxblox_mapper && voxblox_mapper->enabled())
        voxblox_mapper->saveMap();
}

void PoseGraph::loadVoxbloxMap()
{
    std::lock_guard<std::mutex> lock(m_voxblox);
    if (voxblox_mapper && voxblox_mapper->enabled())
        voxblox_mapper->loadMap();
}


void PoseGraph::loadKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    cur_kf->index = global_index;
    global_index++;
    int loop_index = -1;
    bool queue_pose_optimization = false;
    if (flag_detect_loop)
       loop_index = detectLoop(cur_kf, cur_kf->index);
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }
    if (loop_index != -1)
    {
        printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        KeyFrame* old_kf = getKeyFrame(loop_index);
        if (cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;
            queue_pose_optimization = true;
        }
    }
    m_keyframelist.lock();
    Vector3d P;
    Matrix3d R;
    cur_kf->getPose(P, R);
    Quaterniond Q{R};
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    base_path.poses.push_back(pose_stamped);
    base_path.header = pose_stamped.header;

    //draw local connection
    if (SHOW_S_EDGE)
    {
        list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
        for (int i = 0; i < 1; i++)
        {
            if (rit == keyframelist.rend())
                break;
            Vector3d conncected_P;
            Matrix3d connected_R;
            if((*rit)->sequence == cur_kf->sequence)
            {
                (*rit)->getPose(conncected_P, connected_R);
                posegraph_visualization->add_edge(P, conncected_P);
            }
            rit++;
        }
    }
    /*
    if (cur_kf->has_loop)
    {
        KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
        Vector3d connected_P;
        Matrix3d connected_R;
        connected_KF->getPose(connected_P,  connected_R);
        posegraph_visualization->add_loopedge(P, connected_P, SHIFT);
    }
    */

    keyframelist.push_back(cur_kf);
    //publish();
    m_keyframelist.unlock();
    if (queue_pose_optimization)
    {
        m_optimize_buf.lock();
        optimize_buf.push(cur_kf->index);
        m_optimize_buf.unlock();
    }
}

KeyFrame* PoseGraph::getKeyFrame(int index)
{
//    unique_lock<mutex> lock(m_keyframelist);
    list<KeyFrame*>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)
    {
        if((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return NULL;
}

int PoseGraph::detectLoop(KeyFrame* keyframe, int frame_index)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    // set false, not used
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[frame_index] = compressed_image;
    }
    TicToc tmp_t;
    //first query; then add this frame into database!
    QueryResults ret;
    TicToc t_query;
    db.query(keyframe->brief_descriptors, ret, 4, frame_index - 50);
    //printf("query time: %f", t_query.toc());
    //cout << "Searching for Image " << frame_index << ". " << ret << endl;

    TicToc t_add;
    db.add(keyframe->brief_descriptors);
    //printf("add feature time: %f", t_add.toc());
    //------------------------------------------------------------------------------
    // ret[0] is the nearest neighbour's score. threshold change with neighour score
    //------------------------------------------------------------------------------
    bool find_loop = false;
    cv::Mat loop_result;
    if (DEBUG_IMAGE)
    {
        loop_result = compressed_image.clone();
        if (ret.size() > 0)
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
    }
    // visual loop result
    if (DEBUG_IMAGE)
    {
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;
            auto it = image_pool.find(tmp_index);
            cv::Mat tmp_image = (it->second).clone();
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            cv::hconcat(loop_result, tmp_image, loop_result);
        }
    }
    // a good match with its nerghbour
    if (ret.size() >= 1 &&ret[0].Score > 0.05)
    {
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            //if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > 0.015)
            {
                find_loop = true;
                int tmp_index = ret[i].Id;
                if (DEBUG_IMAGE && 0)
                {
                    auto it = image_pool.find(tmp_index);
                    cv::Mat tmp_image = (it->second).clone();
                    putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
                    cv::hconcat(loop_result, tmp_image, loop_result);
                }
            }

        }
    }
/*


    if (DEBUG_IMAGE)
    {
        cv::imshow("loop_result", loop_result);
        cv::waitKey(20);
    }
*/
    if (find_loop && frame_index > 50)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;
    }
    else
        return -1;

}

void PoseGraph::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[keyframe->index] = compressed_image;
    }

    db.add(keyframe->brief_descriptors);
}

void PoseGraph::optimize4DoF()
{
    while(ros::ok())
    {
        int cur_index = -1;
        int first_looped_index = -1;
        m_optimize_buf.lock();
        while(!optimize_buf.empty())
        {
            cur_index = optimize_buf.front();
            first_looped_index = earliest_loop_index;
            optimize_buf.pop();
        }
        m_optimize_buf.unlock();
        if (cur_index != -1)
        {
            printf("optimize pose graph \n");
            TicToc tmp_t;
            m_keyframelist.lock();
            KeyFrame* cur_kf = getKeyFrame(cur_index);
            if (cur_kf == NULL)
            {
                m_keyframelist.unlock();
                std::chrono::milliseconds dura(2000);
                std::this_thread::sleep_for(dura);
                continue;
            }
            if (first_looped_index < 0)
                first_looped_index = 0;

            int max_length = cur_index + 1;

            // w^t_i   w^q_i
            double t_array[max_length][3];
            Quaterniond q_array[max_length];
            double euler_array[max_length][3];
            double sequence_array[max_length];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);
            //loss_function = new ceres::CauchyLoss(1.0);
            ceres::LocalParameterization* angle_local_parameterization =
                AngleLocalParameterization::Create();

            list<KeyFrame*>::iterator it;
            std::vector<KeyFrame*> optimized_keyframes;

            int i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                (*it)->local_index = i;
                optimized_keyframes.push_back(*it);
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r);
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i] = tmp_q;

                Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
                euler_array[i][0] = euler_angle.x();
                euler_array[i][1] = euler_angle.y();
                euler_array[i][2] = euler_angle.z();

                sequence_array[i] = (*it)->sequence;

                problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);

                if ((*it)->index == first_looped_index || (*it)->sequence == 0)
                {
                    problem.SetParameterBlockConstant(euler_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }

                //add edge
                for (int j = 1; j < 5; j++)
                {
                  if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
                  {
                    Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                    Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                    relative_t = q_array[i-j].inverse() * relative_t;
                    double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                    ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                   relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, NULL, euler_array[i-j],
                                            t_array[i-j],
                                            euler_array[i],
                                            t_array[i]);
                  }
                }

                //add loop edge

                if((*it)->has_loop)
                {
                    assert((*it)->loop_index >= first_looped_index);
                    int connected_index = getKeyFrame((*it)->loop_index)->local_index;
                    Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
                    Vector3d relative_t;
                    relative_t = (*it)->getLoopRelativeT();
                    double relative_yaw = (*it)->getLoopRelativeYaw();
                    ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                               relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[connected_index],
                                                                  t_array[connected_index],
                                                                  euler_array[i],
                                                                  t_array[i]);

                }

                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            std::vector<PoseGraphDepthMapEdge> depth_edges =
                buildPoseGraphDepthMapEdges(optimized_keyframes, q_array, t_array);
            if (!depth_edges.empty())
            {
                ceres::LossFunction *depth_loss = DEPTH_MAP_HUBER > 0 ? new ceres::HuberLoss(DEPTH_MAP_HUBER) : NULL;
                double avg_abs_distance = 0.0;
                for (const PoseGraphDepthMapEdge &edge : depth_edges)
                {
                    int target_index = edge.target_local_index;
                    ceres::CostFunction* cost_function = DepthToMapFourDOFError::Create(
                        edge.point_c, edge.plane, edge.scale, DEPTH_MAP_WEIGHT,
                        euler_array[target_index][1], euler_array[target_index][2], qi_d, ti_d);
                    problem.AddResidualBlock(cost_function, depth_loss,
                                             euler_array[target_index],
                                             t_array[target_index]);
                    avg_abs_distance += edge.abs_distance;
                }
                avg_abs_distance /= static_cast<double>(depth_edges.size());
                ROS_INFO("pose graph depth-to-map edges: %lu avg_abs_dist: %.4f",
                         depth_edges.size(), avg_abs_distance);
            }
            m_keyframelist.unlock();

            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";

            //printf("pose optimization time: %f \n", tmp_t.toc());
            /*
            for (int j = 0 ; j < i; j++)
            {
                printf("optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            }
            */
            m_keyframelist.lock();
            i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                Quaterniond tmp_q;
                tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (*it)-> updatePose(tmp_t, tmp_r);

                if ((*it)->index == cur_index)
                    break;
                i++;
            }

            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);
            cur_kf->getVioPose(vio_t, vio_r);
            m_drift.lock();
            yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
            r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();
            //cout << "t_drift " << t_drift.transpose() << endl;
            //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;
            //cout << "yaw drift " << yaw_drift << endl;

            it++;
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift;
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            m_keyframelist.unlock();
            updatePath();
        }

        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
}

void PoseGraph::updatePath()
{
    TicToc t_update;
    // store info for updating dense pcl without locking keyframe list
    // which may take much time
    vector<vector<cv::Point3f>> tmp_keyframelist;
    vector<vector<cv::Vec3b>> tmp_keyframe_colorlist;
    vector<vector<cv::Point3f>> tmp_voxblox_keyframelist;
    vector<vector<cv::Vec3b>> tmp_voxblox_colorlist;
    vector<pair<Matrix3d, Vector3d>> tmp_RTlist;
    std_msgs::Header tmp_header;
    bool has_voxblox_points = false;


    m_keyframelist.lock();
    list<KeyFrame*>::iterator it;
    for (int i = 1; i <= sequence_cnt; i++)
    {
        path[i].poses.clear();
    }
    base_path.poses.clear();
    posegraph_visualization->reset();
    // not used
    if (SAVE_LOOP_PATH)
    {
        ofstream loop_path_file_tmp(VINS_RESULT_PATH, ios::out);
        loop_path_file_tmp.close();
    }

    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getPose(P, R);

        tmp_RTlist.push_back(make_pair(R, P));

        Quaterniond Q;
        Q = R;
//        printf("path p: %f, %f, %f\n",  P.x(),  P.z(),  P.y() );

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time((*it)->time_stamp);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
        pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();

        tmp_keyframelist.push_back((*it)->point_3d_depth_raw);
        tmp_keyframe_colorlist.push_back((*it)->point_3d_depth_color_raw);
        tmp_voxblox_keyframelist.push_back((*it)->point_3d_depth_raw);
        tmp_voxblox_colorlist.push_back((*it)->point_3d_depth_color_raw);
        if (!(*it)->point_3d_depth_raw.empty())
            has_voxblox_points = true;



        if((*it)->sequence == 0)
        {
            base_path.poses.push_back(pose_stamped);
            base_path.header = pose_stamped.header;
        }
        else
        {
            path[(*it)->sequence].poses.push_back(pose_stamped);
            path[(*it)->sequence].header = pose_stamped.header;
            tmp_header = pose_stamped.header;
        }
        //not used
        if (SAVE_LOOP_PATH)
        {
            ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
            loop_path_file.setf(ios::fixed, ios::floatfield);
            loop_path_file.precision(0);
            loop_path_file << (*it)->time_stamp * 1e9 << ",";
            loop_path_file.precision(5);
            loop_path_file  << P.x() << ","
                  << P.y() << ","
                  << P.z() << ","
                  << Q.w() << ","
                  << Q.x() << ","
                  << Q.y() << ","
                  << Q.z() << ","
                  << endl;
            loop_path_file.close();
        }
        //draw local connection
        // not used
        if (SHOW_S_EDGE)
        {
            list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
            list<KeyFrame*>::reverse_iterator lrit;
            for (; rit != keyframelist.rend(); rit++)
            {
                if ((*rit)->index == (*it)->index)
                {
                    lrit = rit;
                    lrit++;
                    for (int i = 0; i < 4; i++)
                    {
                        if (lrit == keyframelist.rend())
                            break;
                        if((*lrit)->sequence == (*it)->sequence)
                        {
                            Vector3d conncected_P;
                            Matrix3d connected_R;
                            (*lrit)->getPose(conncected_P, connected_R);
                            posegraph_visualization->add_edge(P, conncected_P);
                        }
                        lrit++;
                    }
                    break;
                }
            }
        }
        if (SHOW_L_EDGE)
        {
            if ((*it)->has_loop && (*it)->sequence == sequence_cnt)
            {

                KeyFrame* connected_KF = getKeyFrame((*it)->loop_index);
                Vector3d connected_P;
                Matrix3d connected_R;
                connected_KF->getPose(connected_P, connected_R);
                //(*it)->getVioPose(P, R);
                (*it)->getPose(P, R);
                if((*it)->sequence > 0)
                {
                    posegraph_visualization->add_loopedge(P, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
                }
            }
        }

    }
    publish();
    m_keyframelist.unlock();

    if (voxblox_mapper && voxblox_mapper->enabled() && has_voxblox_points)
    {
        ros::Time map_stamp = tmp_header.stamp.isZero() ? ros::Time::now() : tmp_header.stamp;
        std::lock_guard<std::mutex> lock(m_voxblox);
        voxblox_mapper->rebuild(tmp_voxblox_keyframelist, tmp_voxblox_colorlist,
                                tmp_RTlist, qi_d, ti_d, map_stamp);
    }
    
    // throw the costy part beyond m_keyframelist
    sensor_msgs::PointCloud2 rebuilt_pcl;
    bool publish_rebuilt_pcl = false;
    m_octree.lock();
    //some clean up
    octree->deleteTree();
    cloud->clear();
    color_cloud->clear();
    save_cloud->clear();
    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();
    octree->defineBoundingBox(-100, -100, -100, 100, 100, 100);
    int update_count = 0;
    int accepted_count = 0;
    for (size_t kf_idx = 0; kf_idx < tmp_keyframelist.size(); ++kf_idx)
    {
        auto &pcl_vect = tmp_keyframelist[kf_idx];
        Vector3d P;
        Matrix3d R;
        R = tmp_RTlist[kf_idx].first;
        P = tmp_RTlist[kf_idx].second;
        const auto &color_vect = tmp_keyframe_colorlist[kf_idx];
        for (size_t point_idx = 0; point_idx < pcl_vect.size(); ++point_idx)
        {
            const cv::Point3f &pcl = pcl_vect[point_idx];
            Vector3d pts_i(pcl.x , pcl.y, pcl.z);
            Vector3d w_pts_i = R * (qi_d * pts_i + ti_d) + P;
            pcl::PointXYZ searchPoint;
            searchPoint.x = w_pts_i(0);
            searchPoint.y = w_pts_i(1);
            searchPoint.z = w_pts_i(2);
            ++update_count;
            if (octree->getVoxelDensityAtPoint(searchPoint) >= 5)
                continue;
            octree->addPointToCloud(searchPoint, cloud);
            const cv::Vec3b bgr = point_idx < color_vect.size() ?
                color_vect[point_idx] : cv::Vec3b(128, 128, 128);
            const pcl::PointXYZRGB color_point = makeColorPoint(w_pts_i, bgr);
            color_cloud->push_back(color_point);
            save_cloud->push_back(color_point);
            ++accepted_count;
        }
    }
    if (PCL_FILTER_MIN_DENSITY > 1)
        pclFilter(true);
    const int filtered_count = color_cloud ? static_cast<int>(color_cloud->size()) : 0;
    if (color_cloud && !color_cloud->empty())
    {
        pcl::toROSMsg(*color_cloud, rebuilt_pcl);
        rebuilt_pcl.header.stamp = tmp_header.stamp.isZero() ? ros::Time::now() : tmp_header.stamp;
        rebuilt_pcl.header.frame_id = "world";
        publish_rebuilt_pcl = true;
    }
    m_octree.unlock();
    if (publish_rebuilt_pcl)
        pub_octree.publish(rebuilt_pcl);
    ROS_INFO("Update done! Time cost: %f   raw points: %d accepted points: %d filtered points: %d min_density: %d",
             t_update.toc(), update_count, accepted_count, filtered_count, PCL_FILTER_MIN_DENSITY);

}

void PoseGraph::pclFilter(bool flag)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_copy(*(octree->getInputCloud()));
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud_copy(*color_cloud);
    pcl::octree::OctreePointCloudDensity<pcl::PointXYZ>* temp_octree = new pcl::octree::OctreePointCloudDensity<pcl::PointXYZ>(RESOLUTION);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_color_cloud =
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    cout<<"Size of Cloud before filter:"<<cloud_copy.size()<<endl;
    for (size_t point_idx = 0; point_idx < cloud_copy.size(); ++point_idx)
    {
        if (static_cast<int>(octree->getVoxelDensityAtPoint(cloud_copy[point_idx])) >= PCL_FILTER_MIN_DENSITY)
        {
            temp_cloud->push_back(cloud_copy[point_idx]);
            if (point_idx < color_cloud_copy.size())
                temp_color_cloud->push_back(color_cloud_copy[point_idx]);
            else
            {
                pcl::PointXYZRGB color_point;
                color_point.x = cloud_copy[point_idx].x;
                color_point.y = cloud_copy[point_idx].y;
                color_point.z = cloud_copy[point_idx].z;
                color_point.r = color_point.g = color_point.b = 128;
                temp_color_cloud->push_back(color_point);
            }
        }
    }
    temp_octree->defineBoundingBox(-100, -100, -100, 100, 100, 100);
    temp_octree->setInputCloud(temp_cloud);
    temp_octree->addPointsFromInputCloud();
    cout<<"Size of Cloud after filter:"<<temp_cloud->size()<<endl;
    // filter octree after loop
    if (flag)
    {
        //lock octree and cloud
        delete octree;
        (*cloud).clear();
        octree = temp_octree;
        cloud = temp_cloud;
        color_cloud = temp_color_cloud;
        //sensor_msgs::PointCloud2 tmp_pcl;
        //pcl::toROSMsg(*temp_cloud, tmp_pcl);
        //tmp_pcl.header.stamp =  ros::Time::now();
        //tmp_pcl.header.frame_id = "world";
        //pub_octree.publish(tmp_pcl);
    }
    // use 'p' to filter octree at any time
    else
    {
        sensor_msgs::PointCloud2 tmp_pcl;
        pcl::toROSMsg(*temp_color_cloud, tmp_pcl);
        tmp_pcl.header.stamp =  ros::Time::now();
        tmp_pcl.header.frame_id = "world";
        pub_octree.publish(tmp_pcl);
    }


}

void PoseGraph::savePoseGraph()
{
    m_keyframelist.lock();
    TicToc tmp_t;
    FILE *pFile,*pFile_shan_pg,*pFile_shan_vio;
    printf("pose graph path: %s\n",POSE_GRAPH_SAVE_PATH.c_str());
    printf("pose graph saving... \n");
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
//    string file_path_shan_pg = POSE_GRAPH_SAVE_PATH + "stamped_traj_estimate_mono_pg.txt";
//    string file_path_shan_vio = POSE_GRAPH_SAVE_PATH + "stamped_traj_estimate_mono_vio.txt";
    string file_path_shan_pg = "/home/shanzy/rpg_trajectory_evaluation/results/laptop/vio_mono/laptop_vio_mono_MH_01/stamped_traj_estimate.txt";
    string file_path_shan_vio = "/home/shanzy/rpg_trajectory_evaluation/results/laptop/vio_mono/laptop_vio_mono_MH_01/vio_stamped_traj_estimate.txt";
    pFile = fopen (file_path.c_str(),"w");
    pFile_shan_pg = fopen(file_path_shan_pg.c_str(),"w");
    pFile_shan_vio = fopen(file_path_shan_vio.c_str(),"w");
    if (!pFile)
    {
        ROS_ERROR("Failed to open pose graph save file: %s", file_path.c_str());
        m_keyframelist.unlock();
        return;
    }
    if (!pFile_shan_pg)
        ROS_WARN("Skipping legacy pose graph trajectory export: %s", file_path_shan_pg.c_str());
    if (!pFile_shan_vio)
        ROS_WARN("Skipping legacy VIO trajectory export: %s", file_path_shan_vio.c_str());
    //fprintf(pFile, "index time_stamp Tx Ty Tz Qw Qx Qy Qz loop_index loop_info\n");
    list<KeyFrame*>::iterator it;
    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        std::string image_path, descriptor_path, brief_path, keypoints_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_image.png";
            imwrite(image_path.c_str(), (*it)->image);
        }
        Quaterniond VIO_tmp_Q{(*it)->vio_R_w_i};
        Quaterniond PG_tmp_Q{(*it)->R_w_i};
        Quaterniond rVIO_tmp_Q{(*it)->vio_R_w_i.transpose()};
        Quaterniond rPG_tmp_Q{(*it)->R_w_i.transpose()};
        Vector3d VIO_tmp_T = (*it)->vio_T_w_i;
        Vector3d PG_tmp_T = (*it)->T_w_i;

        fprintf (pFile, " %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %d\n",(*it)->index, (*it)->time_stamp,
                 VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(),
                 PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(),
                 VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(),
                 PG_tmp_Q.w(), PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(),
                 (*it)->loop_index,
                 (*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2), (*it)->loop_info(3),
                 (*it)->loop_info(4), (*it)->loop_info(5), (*it)->loop_info(6), (*it)->loop_info(7),
                 (int)(*it)->keypoints.size());
        if (pFile_shan_pg)
            fprintf (pFile_shan_pg, "%f %f %f %f %f %f %f %f\n",(*it)->time_stamp,
                     PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(),
                     PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(),PG_tmp_Q.w());
        if (pFile_shan_vio)
            fprintf (pFile_shan_vio, "%f %f %f %f %f %f %f %f\n",(*it)->time_stamp,
                     VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(),
                     VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(),VIO_tmp_Q.w());

        // write keypoints, brief_descriptors   vector<cv::KeyPoint> keypoints vector<BRIEF::bitset> brief_descriptors;
        assert((*it)->keypoints.size() == (*it)->brief_descriptors.size());
        brief_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_briefdes.dat";
        std::ofstream brief_file(brief_path, std::ios::binary);
        keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "w");
        if (!keypoints_file)
            ROS_WARN("Failed to open keypoints save file: %s", keypoints_path.c_str());
        for (int i = 0; i < (int)(*it)->keypoints.size(); i++)
        {
            brief_file << (*it)->brief_descriptors[i] << endl;
            if (keypoints_file)
                fprintf(keypoints_file, "%f %f %f %f\n", (*it)->keypoints[i].pt.x, (*it)->keypoints[i].pt.y,
                        (*it)->keypoints_norm[i].pt.x, (*it)->keypoints_norm[i].pt.y);
        }
        brief_file.close();
        if (keypoints_file)
            fclose(keypoints_file);
    }
    if (pFile_shan_vio)
        fclose(pFile_shan_vio);
    if (pFile_shan_pg)
        fclose(pFile_shan_pg);
    fclose(pFile);


    printf("save pose graph time: %f s\n", tmp_t.toc() / 1000);
    m_keyframelist.unlock();
}

void PoseGraph::loadPoseGraph()
{
    TicToc tmp_t;
    FILE * pFile;
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    pFile = fopen (file_path.c_str(),"r");
    if (pFile == NULL)
    {
        printf("lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return;
    }
    int index;
    double time_stamp;
    double VIO_Tx, VIO_Ty, VIO_Tz;
    double PG_Tx, PG_Ty, PG_Tz;
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    int loop_index;
    int keypoints_num;
    Eigen::Matrix<double, 8, 1 > loop_info;
    int cnt = 0;
    while (fscanf(pFile,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d", &index, &time_stamp,
                                    &VIO_Tx, &VIO_Ty, &VIO_Tz,
                                    &PG_Tx, &PG_Ty, &PG_Tz,
                                    &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz,
                                    &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz,
                                    &loop_index,
                                    &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3,
                                    &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                                    &keypoints_num) != EOF)
    {
        /*
        printf("I read: %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d\n", index, time_stamp,
                                    VIO_Tx, VIO_Ty, VIO_Tz,
                                    PG_Tx, PG_Ty, PG_Tz,
                                    VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz,
                                    PG_Qw, PG_Qx, PG_Qy, PG_Qz,
                                    loop_index,
                                    loop_info_0, loop_info_1, loop_info_2, loop_info_3,
                                    loop_info_4, loop_info_5, loop_info_6, loop_info_7,
                                    keypoints_num);
        */
        cv::Mat image;
        std::string image_path, descriptor_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_image.png";
            image = cv::imread(image_path.c_str(), 0);
        }

        Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Quaterniond VIO_Q;
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Matrix3d VIO_R, PG_R;
        VIO_R = VIO_Q.toRotationMatrix();
        PG_R = PG_Q.toRotationMatrix();
        Eigen::Matrix<double, 8, 1 > loop_info;
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;

        if (loop_index != -1)
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
            {
                earliest_loop_index = loop_index;
            }

        // load keypoints, brief_descriptors
        string brief_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_briefdes.dat";
        std::ifstream brief_file(brief_path, std::ios::binary);
        string keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "r");
        vector<cv::KeyPoint> keypoints;
        vector<cv::KeyPoint> keypoints_norm;
        vector<BRIEF::bitset> brief_descriptors;
        for (int i = 0; i < keypoints_num; i++)
        {
            BRIEF::bitset tmp_des;
            brief_file >> tmp_des;
            brief_descriptors.push_back(tmp_des);
            cv::KeyPoint tmp_keypoint;
            cv::KeyPoint tmp_keypoint_norm;
            double p_x, p_y, p_x_norm, p_y_norm;
            if(!fscanf(keypoints_file,"%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm))
                printf(" fail to load pose graph \n");
            tmp_keypoint.pt.x = p_x;
            tmp_keypoint.pt.y = p_y;
            tmp_keypoint_norm.pt.x = p_x_norm;
            tmp_keypoint_norm.pt.y = p_y_norm;
            keypoints.push_back(tmp_keypoint);
            keypoints_norm.push_back(tmp_keypoint_norm);
        }
        brief_file.close();
        fclose(keypoints_file);

        KeyFrame* keyframe = new KeyFrame(time_stamp, index, VIO_T, VIO_R, PG_T, PG_R, image, loop_index, loop_info, keypoints, keypoints_norm, brief_descriptors);
        loadKeyFrame(keyframe, 0);
        if (cnt % 20 == 0)
        {
            publish();
        }
        cnt++;
    }
    fclose (pFile);
    printf("load pose graph time: %f s\n", tmp_t.toc()/1000);
    base_sequence = 0;
}

void PoseGraph::publish()
{
    for (int i = 1; i <= sequence_cnt; i++)
    {
        //if (sequence_loop[i] == true || i == base_sequence)
        if (1 || i == base_sequence)
        {
            pub_pg_path.publish(path[i]);
            pub_path[i].publish(path[i]);
            //posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
        }
    }

    pub_base_path.publish(base_path);

    //posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
}

void PoseGraph::updateKeyFrameLoop(int index, Eigen::Matrix<double, 8, 1 > &_loop_info)
{
    KeyFrame* kf = getKeyFrame(index);
    kf->updateLoop(_loop_info);
    if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0)
    {
        if (FAST_RELOCALIZATION)
        {
            KeyFrame* old_kf = getKeyFrame(kf->loop_index);
            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getPose(w_P_old, w_R_old);
            kf->getVioPose(vio_P_cur, vio_R_cur);

            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = kf->getLoopRelativeT();
            relative_q = (kf->getLoopRelativeQ()).toRotationMatrix();
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t;
            shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
            shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur;

            m_drift.lock();
            yaw_drift = shift_yaw;
            r_drift = shift_r;
            t_drift = shift_t;
            m_drift.unlock();
        }
    }
}
