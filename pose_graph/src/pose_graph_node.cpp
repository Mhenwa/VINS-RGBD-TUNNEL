#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <iostream>
#include <ros/package.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#define SKIP_FIRST_CNT 10
using namespace std;

queue<sensor_msgs::ImageConstPtr> image_buf,depth_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;

float PCL_MAX_DIST, PCL_MIN_DIST, RESOLUTION;
int PCL_FILTER_MIN_DENSITY = 2;
int U_BOUNDARY, D_BOUNDARY, L_BOUNDARY, R_BOUNDARY;
int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int PCL_DIST;
int DEBUG_IMAGE;
int VISUALIZE_IMU_FORWARD;
int LOOP_CLOSURE;
int FAST_RELOCALIZATION;
int DENSE_DEPTH_MEDIAN_KERNEL = 3;
int DENSE_DEPTH_EDGE_FILTER = 1;
double DENSE_DEPTH_EDGE_THRESHOLD = 0.20;
int DENSE_DEPTH_ADAPTIVE_SAMPLING = 1;
int DENSE_DEPTH_NEAR_STRIDE = 5;
int DENSE_DEPTH_MID_STRIDE = 10;
int DENSE_DEPTH_FAR_STRIDE = 15;
double DENSE_DEPTH_NEAR_RANGE = 2.0;
double DENSE_DEPTH_MID_RANGE = 4.0;
int DENSE_DEPTH_MAX_POINTS_PER_KEYFRAME = 12000;
int DENSE_DEPTH_PROFILE = 0;
int AUTO_SAVE_MAP_ON_EXIT = 1;
int USE_DEPTH_TO_MAP_POSE_GRAPH = 1;
double DEPTH_MAP_WEIGHT = 100.0;
double DEPTH_MAP_HUBER = 1.0;
int DEPTH_MAP_MIN_EDGES = 50;
int DEPTH_MAP_MAX_EDGES_PER_FRAME = 800;
int DEPTH_MAP_NEIGHBOR_COUNT = 5;
double DEPTH_MAP_MAX_NEIGHBOR_DIST = 1.0;
double DEPTH_MAP_PLANE_MAX_DIST = 0.2;
double DEPTH_MAP_MIN_SCALE = 0.1;
int DEPTH_MAP_POSE_GRAPH_TARGET_KEYFRAMES = 5;
int DEPTH_MAP_POSE_GRAPH_OPT_INTERVAL = 10;


camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;

Eigen::Matrix<double, 3, 1> ti_d;
Eigen::Matrix<double, 3, 3> qi_d;

ros::Publisher pub_match_img;
ros::Publisher pub_match_points;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_key_odometrys;
ros::Publisher pub_vio_path;
nav_msgs::Path no_loop_path;

template <typename T>
void readOptionalRosParam(ros::NodeHandle &n, const std::string &name, T &value)
{
    T override_value;
    if (n.getParam(name, override_value))
    {
        value = override_value;
        ROS_INFO_STREAM("Override " << name << ": " << value);
    }
}

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_PATH;
std::string PCD_OUTPUT_PATH;

float depthMetersAt(const cv::Mat &depth, int row, int col)
{
    if (row < 0 || row >= depth.rows || col < 0 || col >= depth.cols)
        return 0.0f;
    return static_cast<float>(depth.at<unsigned short>(row, col)) / 1000.0f;
}

bool validDepth(float depth)
{
    return std::isfinite(depth) && depth > PCL_MIN_DIST && depth < PCL_MAX_DIST;
}

bool isDepthDiscontinuity(const cv::Mat &depth, int row, int col, float center_depth)
{
    if (!DENSE_DEPTH_EDGE_FILTER)
        return false;

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    for (int k = 0; k < 4; ++k)
    {
        const float neighbor_depth = depthMetersAt(depth, row + dr[k], col + dc[k]);
        if (validDepth(neighbor_depth) &&
            std::abs(neighbor_depth - center_depth) > DENSE_DEPTH_EDGE_THRESHOLD)
        {
            return true;
        }
    }
    return false;
}

int denseDepthStride(float depth)
{
    if (!DENSE_DEPTH_ADAPTIVE_SAMPLING)
        return std::max(1, PCL_DIST);
    if (depth < DENSE_DEPTH_NEAR_RANGE)
        return std::max(1, DENSE_DEPTH_NEAR_STRIDE);
    if (depth < DENSE_DEPTH_MID_RANGE)
        return std::max(1, DENSE_DEPTH_MID_STRIDE);
    return std::max(1, DENSE_DEPTH_FAR_STRIDE);
}

void limitDenseDepthPoints(vector<cv::Point3f> &points, vector<cv::Vec3b> &colors)
{
    if (DENSE_DEPTH_MAX_POINTS_PER_KEYFRAME <= 0 ||
        static_cast<int>(points.size()) <= DENSE_DEPTH_MAX_POINTS_PER_KEYFRAME)
    {
        return;
    }

    const size_t keep = static_cast<size_t>(DENSE_DEPTH_MAX_POINTS_PER_KEYFRAME);
    const size_t original_size = points.size();
    vector<cv::Point3f> limited_points;
    vector<cv::Vec3b> limited_colors;
    limited_points.reserve(keep);
    limited_colors.reserve(keep);
    for (size_t out_idx = 0; out_idx < keep; ++out_idx)
    {
        const size_t src_idx = out_idx * original_size / keep;
        limited_points.push_back(points[src_idx]);
        if (src_idx < colors.size())
            limited_colors.push_back(colors[src_idx]);
        else
            limited_colors.push_back(cv::Vec3b(128, 128, 128));
    }
    points.swap(limited_points);
    colors.swap(limited_colors);
}

bool savePoseGraphOutputs(std::string *message)
{
    if (!LOOP_CLOSURE)
    {
        if (message)
            *message = "loop closure is disabled; pose graph outputs are not active";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_process);
    posegraph.savePoseGraph();
    posegraph.saveVoxbloxMap();
    if (message)
        *message = "saved pose graph and Voxblox outputs";
    return true;
}

bool saveMapService(std_srvs::Trigger::Request & /*request*/,
                    std_srvs::Trigger::Response &response)
{
    response.success = savePoseGraphOutputs(&response.message);
    return true;
}
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

string trimTrailingSlash(string path)
{
    while (path.size() > 1 && path.back() == '/')
        path.pop_back();
    return path;
}

string parentPath(const string &path)
{
    string clean_path = trimTrailingSlash(path);
    size_t slash_pos = clean_path.find_last_of('/');
    if (slash_pos == string::npos)
        return "";
    if (slash_pos == 0)
        return "/";
    return clean_path.substr(0, slash_pos);
}

string joinPath(const string &base, const string &name)
{
    if (base.empty())
        return name;
    if (base.back() == '/')
        return base + name;
    return base + "/" + name;
}

bool directoryExists(const string &path)
{
    struct stat info;
    return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

bool makeDirectoryRecursive(const string &path)
{
    if (path.empty() || directoryExists(path))
        return true;

    string parent = parentPath(path);
    if (!parent.empty() && parent != path && !makeDirectoryRecursive(parent))
        return false;

    if (mkdir(path.c_str(), 0755) == 0 || errno == EEXIST)
        return directoryExists(path);

    return false;
}

//not used in my case, just ignore sequence 1-5
void new_sequence()
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences.");
        ROS_BREAK();
    }
    posegraph.posegraph_visualization->reset();
    posegraph.publish();
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!depth_buf.empty())
        depth_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{
    //ROS_WARN("image_callback!1");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    image_buf.push(image_msg);
    depth_buf.push(depth_msg);
    m_buf.unlock();
    //printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();
    else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 || image_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = image_msg->header.stamp.toSec();
}

void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    //ROS_INFO("point_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
    /*
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x,
                                                     point_msg->points[i].y,
                                                     point_msg->points[i].z,
                                                     point_msg->channels[i].values[0],
                                                     point_msg->channels[i].values[1]);
    }
    */
}

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("pose_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
    /*
    printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                       pose_msg->pose.pose.position.y,
                                                       pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z);
    */
}

// not used
void imu_forward_callback(const nav_msgs::Odometry::ConstPtr &forward_msg)
{
    if (VISUALIZE_IMU_FORWARD)
    {
        Vector3d vio_t(forward_msg->pose.pose.position.x, forward_msg->pose.pose.position.y, forward_msg->pose.pose.position.z);
        Quaterniond vio_q;
        vio_q.w() = forward_msg->pose.pose.orientation.w;
        vio_q.x() = forward_msg->pose.pose.orientation.x;
        vio_q.y() = forward_msg->pose.pose.orientation.y;
        vio_q.z() = forward_msg->pose.pose.orientation.z;

        vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
        vio_q = posegraph.w_r_vio *  vio_q;

        vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
        vio_q = posegraph.r_drift * vio_q;

        Vector3d vio_t_cam;
        Quaterniond vio_q_cam;
        vio_t_cam = vio_t + vio_q * tic;
        vio_q_cam = vio_q * qic;

        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, forward_msg->header);
    }
}
void relo_relative_pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    Vector3d relative_t = Vector3d(pose_msg->pose.pose.position.x,
                                   pose_msg->pose.pose.position.y,
                                   pose_msg->pose.pose.position.z);
    Quaterniond relative_q;
    relative_q.w() = pose_msg->pose.pose.orientation.w;
    relative_q.x() = pose_msg->pose.pose.orientation.x;
    relative_q.y() = pose_msg->pose.pose.orientation.y;
    relative_q.z() = pose_msg->pose.pose.orientation.z;
    double relative_yaw = pose_msg->twist.twist.linear.x;
    int index = pose_msg->twist.twist.linear.y;
    //printf("receive index %d \n", index );
    Eigen::Matrix<double, 8, 1 > loop_info;
    loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
                 relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                 relative_yaw;
    posegraph.updateKeyFrameLoop(index, loop_info);

}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio *  vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;

    if (!VISUALIZE_IMU_FORWARD)
    {
        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
    }

    odometry_buf.push(vio_t_cam);
    if (odometry_buf.size() > 10)
    {
        odometry_buf.pop();
    }

    visualization_msgs::Marker key_odometrys;
    key_odometrys.header = pose_msg->header;
    key_odometrys.header.frame_id = "world";
    key_odometrys.ns = "key_odometrys";
    key_odometrys.type = visualization_msgs::Marker::SPHERE_LIST;
    key_odometrys.action = visualization_msgs::Marker::ADD;
    key_odometrys.pose.orientation.w = 1.0;
    key_odometrys.lifetime = ros::Duration();

    //static int key_odometrys_id = 0;
    key_odometrys.id = 0; //key_odometrys_id++;
    key_odometrys.scale.x = 0.1;
    key_odometrys.scale.y = 0.1;
    key_odometrys.scale.z = 0.1;
    key_odometrys.color.r = 1.0;
    key_odometrys.color.a = 1.0;

    for (unsigned int i = 0; i < odometry_buf.size(); i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d vio_t;
        vio_t = odometry_buf.front();
        odometry_buf.pop();
        pose_marker.x = vio_t.x();
        pose_marker.y = vio_t.y();
        pose_marker.z = vio_t.z();
        key_odometrys.points.push_back(pose_marker);
        odometry_buf.push(vio_t);
    }
    pub_key_odometrys.publish(key_odometrys);

    // not used
    if (!LOOP_CLOSURE)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = pose_msg->header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = vio_t.x();
        pose_stamped.pose.position.y = vio_t.y();
        pose_stamped.pose.position.z = vio_t.z();
        no_loop_path.header = pose_msg->header;
        no_loop_path.header.frame_id = "world";
        no_loop_path.poses.push_back(pose_stamped);
        pub_vio_path.publish(no_loop_path);
    }
}

void extrinsic_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}

void process()
{
    if (!LOOP_CLOSURE)
        return;
    while (ros::ok())
    {
        sensor_msgs::ImageConstPtr image_msg = NULL;
        sensor_msgs::ImageConstPtr depth_msg = NULL;
        sensor_msgs::PointCloudConstPtr point_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;
        // find out the messages with same time stamp
        m_buf.lock();
        //get image_msg, pose_msg and point_msg
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec()
                && point_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                {
                    image_buf.pop();
                    depth_buf.pop();
                }
                image_msg = image_buf.front();
                depth_msg = depth_buf.front();
                image_buf.pop();
                depth_buf.pop();


                while (point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();
        if (pose_msg != NULL)
        {
            //printf(" pose time %f \n", pose_msg->header.stamp.toSec());
            //printf(" point time %f \n", point_msg->header.stamp.toSec());
            //printf(" image time %f \n", image_msg->header.stamp.toSec());
            // skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

            cv::Mat color_image;
            try
            {
                if (image_msg->encoding == sensor_msgs::image_encodings::BGR8)
                    color_image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image.clone();
                else if (image_msg->encoding == sensor_msgs::image_encodings::RGB8)
                    color_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
                else if (image_msg->encoding == sensor_msgs::image_encodings::BGRA8)
                    cv::cvtColor(cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGRA8)->image,
                                 color_image, cv::COLOR_BGRA2BGR);
                else if (image_msg->encoding == sensor_msgs::image_encodings::RGBA8)
                    cv::cvtColor(cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::RGBA8)->image,
                                 color_image, cv::COLOR_RGBA2BGR);
                else if (image_msg->encoding == "8UC3")
                    color_image = cv_bridge::toCvShare(image_msg)->image.clone();
                else
                    cv::cvtColor(ptr->image, color_image, cv::COLOR_GRAY2BGR);
            }
            catch (const cv_bridge::Exception &e)
            {
                ROS_WARN("Failed to convert image to BGR for dense map color: %s", e.what());
                cv::cvtColor(ptr->image, color_image, cv::COLOR_GRAY2BGR);
            }

            //depth has encoding TYPE_16UC1
            cv_bridge::CvImageConstPtr depth_ptr;
            // debug use     std::cout<<depth_msg->encoding<<std::endl;
            {
                sensor_msgs::Image img;
                img.header = depth_msg->header;
                img.height = depth_msg->height;
                img.width = depth_msg->width;
                img.is_bigendian = depth_msg->is_bigendian;
                img.step = depth_msg->step;
                img.data = depth_msg->data;
                img.encoding = sensor_msgs::image_encodings::MONO16;
                depth_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
            }

            cv::Mat image = ptr->image;
            cv::Mat depth = depth_ptr->image;
            // build keyframe
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();
            if((T - last_t).norm() > SKIP_DIS)
            {
                vector<cv::Point3f> point_3d;
                vector<cv::Point2f> point_2d_uv;
                vector<cv::Point2f> point_2d_normal;
                vector<cv::Point3f> point_3d_depth;
                vector<cv::Vec3b> point_3d_depth_color;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    //printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }
                cv::Mat dense_depth = depth;
                if (DENSE_DEPTH_MEDIAN_KERNEL >= 3)
                {
                    if (DENSE_DEPTH_MEDIAN_KERNEL % 2 == 0)
                        DENSE_DEPTH_MEDIAN_KERNEL += 1;
                    cv::medianBlur(depth, dense_depth, DENSE_DEPTH_MEDIAN_KERNEL);
                }

                TicToc dense_depth_timer;
                int depth_candidates = 0;
                int depth_rejected_range = 0;
                int depth_rejected_edge = 0;
                int depth_rejected_stride = 0;
                const int scan_stride = DENSE_DEPTH_ADAPTIVE_SAMPLING ?
                    std::max(1, std::min(DENSE_DEPTH_NEAR_STRIDE,
                                         std::min(DENSE_DEPTH_MID_STRIDE, DENSE_DEPTH_FAR_STRIDE))) :
                    std::max(1, PCL_DIST);

                for (int i = L_BOUNDARY; i < COL - R_BOUNDARY; i += scan_stride)
                {
                    for (int j = U_BOUNDARY; j < ROW - D_BOUNDARY; j += scan_stride)
                    {
                        ++depth_candidates;
                        float depth_val = depthMetersAt(dense_depth, j, i);
                        if (!validDepth(depth_val))
                        {
                            ++depth_rejected_range;
                            continue;
                        }
                        if (isDepthDiscontinuity(dense_depth, j, i, depth_val))
                        {
                            ++depth_rejected_edge;
                            continue;
                        }
                        const int sample_stride = denseDepthStride(depth_val);
                        if (((i - L_BOUNDARY) % sample_stride) != 0 ||
                            ((j - U_BOUNDARY) % sample_stride) != 0)
                        {
                            ++depth_rejected_stride;
                            continue;
                        }

                        Eigen::Vector2d a(i, j);
                        Eigen::Vector3d b;
						//depth is aligned
                        m_camera->liftProjective(a, b);
                        point_3d_depth.push_back(cv::Point3f(b.x() * depth_val, b.y() * depth_val, depth_val));
                        if (j >= 0 && j < color_image.rows && i >= 0 && i < color_image.cols)
                            point_3d_depth_color.push_back(color_image.at<cv::Vec3b>(j, i));
                        else
                            point_3d_depth_color.push_back(cv::Vec3b(128, 128, 128));
                    }
                }
                const int depth_points_before_limit = static_cast<int>(point_3d_depth.size());
                limitDenseDepthPoints(point_3d_depth, point_3d_depth_color);
                if (DENSE_DEPTH_PROFILE && frame_index % 30 == 0)
                {
                    ROS_INFO("dense depth points: %zu/%d candidates, range_reject: %d edge_reject: %d stride_reject: %d limited_from: %d time: %.2f ms",
                             point_3d_depth.size(), depth_candidates,
                             depth_rejected_range, depth_rejected_edge, depth_rejected_stride,
                             depth_points_before_limit, dense_depth_timer.toc());
                }

                // 通过frame_index标记对应帧
                // add sparse depth img to this class
                KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                                   point_3d_depth, point_3d_depth_color,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
                m_process.lock();
                start_flag = 1;
                posegraph.addKeyFrame(keyframe, 1);
                m_process.unlock();
                frame_index++;
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void command()
{
    if (!LOOP_CLOSURE)
        return;
    while(ros::ok())
    {
        char c = getchar();
        if (c == 's')
        {
            savePoseGraphOutputs(NULL);
            printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            printf("program shutting down...\n");
            ros::shutdown();
        }
        if (c == 'n')
            new_sequence();

        if (c == 'p')
        {
            TicToc t_filter;
            posegraph.pclFilter(false);
            printf("pclFilter time: %f", t_filter.toc());
        }
        if (c == 'd')
        {
            TicToc t_pcdfile;
            posegraph.save_cloud->width = posegraph.save_cloud->points.size();
            posegraph.save_cloud->height = 1;
	        pcl::io::savePCDFileASCII(joinPath(PCD_OUTPUT_PATH, "pcd_file_" + to_string(frame_index) + "keyframes.pcd"), *(posegraph.save_cloud));
            printf("Save pcd file done! Time cost: %f", t_pcdfile.toc());
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_graph");
    ros::NodeHandle n("~");
    posegraph.registerPub(n);

    // read param
    n.getParam("visualization_shift_x", VISUALIZATION_SHIFT_X);
    n.getParam("visualization_shift_y", VISUALIZATION_SHIFT_Y);
    n.getParam("skip_cnt", SKIP_CNT);
    n.getParam("skip_dis", SKIP_DIS);
    std::string config_file;
    n.getParam("config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    double camera_visual_size = fsSettings["visualize_camera_size"];
    cameraposevisual.setScale(camera_visual_size);
    cameraposevisual.setLineWidth(camera_visual_size / 10.0);


    LOOP_CLOSURE = fsSettings["loop_closure"];
    std::string IMAGE_TOPIC,DEPTH_TOPIC;
    int LOAD_PREVIOUS_POSE_GRAPH;
    // prepare for loop closure (load vocabulary, set topic, etc)
    if (LOOP_CLOSURE)
    {
        ROW = fsSettings["image_height"];
        COL = fsSettings["image_width"];
        PCL_DIST = fsSettings["pcl_dist"];
        U_BOUNDARY = fsSettings["u_boundary"];
        D_BOUNDARY = fsSettings["d_boundary"];
        L_BOUNDARY = fsSettings["l_boundary"];
        R_BOUNDARY = fsSettings["r_boundary"];
        PCL_MIN_DIST = fsSettings["pcl_min_dist"];
        PCL_MAX_DIST = fsSettings["pcl_max_dist"];
		RESOLUTION = fsSettings["resolution"];
        if (!fsSettings["pcl_filter_min_density"].empty())
            PCL_FILTER_MIN_DENSITY = fsSettings["pcl_filter_min_density"];
        if (PCL_FILTER_MIN_DENSITY < 1)
            PCL_FILTER_MIN_DENSITY = 1;
        if (!fsSettings["dense_depth_median_kernel"].empty())
            DENSE_DEPTH_MEDIAN_KERNEL = fsSettings["dense_depth_median_kernel"];
        if (!fsSettings["dense_depth_edge_filter"].empty())
            DENSE_DEPTH_EDGE_FILTER = fsSettings["dense_depth_edge_filter"];
        if (!fsSettings["dense_depth_edge_threshold"].empty())
            DENSE_DEPTH_EDGE_THRESHOLD = fsSettings["dense_depth_edge_threshold"];
        if (!fsSettings["dense_depth_adaptive_sampling"].empty())
            DENSE_DEPTH_ADAPTIVE_SAMPLING = fsSettings["dense_depth_adaptive_sampling"];
        if (!fsSettings["dense_depth_near_stride"].empty())
            DENSE_DEPTH_NEAR_STRIDE = fsSettings["dense_depth_near_stride"];
        if (!fsSettings["dense_depth_mid_stride"].empty())
            DENSE_DEPTH_MID_STRIDE = fsSettings["dense_depth_mid_stride"];
        if (!fsSettings["dense_depth_far_stride"].empty())
            DENSE_DEPTH_FAR_STRIDE = fsSettings["dense_depth_far_stride"];
        if (!fsSettings["dense_depth_near_range"].empty())
            DENSE_DEPTH_NEAR_RANGE = fsSettings["dense_depth_near_range"];
        if (!fsSettings["dense_depth_mid_range"].empty())
            DENSE_DEPTH_MID_RANGE = fsSettings["dense_depth_mid_range"];
        if (!fsSettings["dense_depth_max_points_per_keyframe"].empty())
            DENSE_DEPTH_MAX_POINTS_PER_KEYFRAME = fsSettings["dense_depth_max_points_per_keyframe"];
        if (!fsSettings["dense_depth_profile"].empty())
            DENSE_DEPTH_PROFILE = fsSettings["dense_depth_profile"];
        if (!fsSettings["auto_save_map_on_exit"].empty())
            AUTO_SAVE_MAP_ON_EXIT = fsSettings["auto_save_map_on_exit"];
        if (DENSE_DEPTH_MEDIAN_KERNEL < 0)
            DENSE_DEPTH_MEDIAN_KERNEL = 0;
        if (DENSE_DEPTH_NEAR_STRIDE < 1)
            DENSE_DEPTH_NEAR_STRIDE = 1;
        if (DENSE_DEPTH_MID_STRIDE < 1)
            DENSE_DEPTH_MID_STRIDE = 1;
        if (DENSE_DEPTH_FAR_STRIDE < 1)
            DENSE_DEPTH_FAR_STRIDE = 1;
        if (DENSE_DEPTH_EDGE_THRESHOLD <= 0.0)
            DENSE_DEPTH_EDGE_THRESHOLD = 0.20;
        if (DENSE_DEPTH_NEAR_RANGE <= 0.0)
            DENSE_DEPTH_NEAR_RANGE = 2.0;
        if (DENSE_DEPTH_MID_RANGE <= DENSE_DEPTH_NEAR_RANGE)
            DENSE_DEPTH_MID_RANGE = DENSE_DEPTH_NEAR_RANGE + 1.0;
        ROS_INFO("dense depth mapping: median_kernel: %d edge_filter: %d edge_threshold: %.3f adaptive: %d strides: %d/%d/%d ranges: %.2f/%.2f max_points: %d profile: %d",
                 DENSE_DEPTH_MEDIAN_KERNEL, DENSE_DEPTH_EDGE_FILTER, DENSE_DEPTH_EDGE_THRESHOLD,
                 DENSE_DEPTH_ADAPTIVE_SAMPLING, DENSE_DEPTH_NEAR_STRIDE, DENSE_DEPTH_MID_STRIDE,
                 DENSE_DEPTH_FAR_STRIDE, DENSE_DEPTH_NEAR_RANGE, DENSE_DEPTH_MID_RANGE,
                 DENSE_DEPTH_MAX_POINTS_PER_KEYFRAME, DENSE_DEPTH_PROFILE);
        ROS_INFO("auto save map on exit: %d", AUTO_SAVE_MAP_ON_EXIT);
        if (!fsSettings["use_depth_to_map_pose_graph"].empty())
            USE_DEPTH_TO_MAP_POSE_GRAPH = fsSettings["use_depth_to_map_pose_graph"];
        if (!fsSettings["depth_map_weight"].empty())
            DEPTH_MAP_WEIGHT = fsSettings["depth_map_weight"];
        if (!fsSettings["depth_map_huber"].empty())
            DEPTH_MAP_HUBER = fsSettings["depth_map_huber"];
        if (!fsSettings["depth_map_min_edges"].empty())
            DEPTH_MAP_MIN_EDGES = fsSettings["depth_map_min_edges"];
        if (!fsSettings["depth_map_max_edges_per_frame"].empty())
            DEPTH_MAP_MAX_EDGES_PER_FRAME = fsSettings["depth_map_max_edges_per_frame"];
        if (!fsSettings["depth_map_neighbor_count"].empty())
            DEPTH_MAP_NEIGHBOR_COUNT = fsSettings["depth_map_neighbor_count"];
        if (!fsSettings["depth_map_max_neighbor_dist"].empty())
            DEPTH_MAP_MAX_NEIGHBOR_DIST = fsSettings["depth_map_max_neighbor_dist"];
        if (!fsSettings["depth_map_plane_max_dist"].empty())
            DEPTH_MAP_PLANE_MAX_DIST = fsSettings["depth_map_plane_max_dist"];
        if (!fsSettings["depth_map_min_scale"].empty())
            DEPTH_MAP_MIN_SCALE = fsSettings["depth_map_min_scale"];
        if (!fsSettings["depth_map_pose_graph_target_keyframes"].empty())
            DEPTH_MAP_POSE_GRAPH_TARGET_KEYFRAMES = fsSettings["depth_map_pose_graph_target_keyframes"];
        if (!fsSettings["depth_map_pose_graph_opt_interval"].empty())
            DEPTH_MAP_POSE_GRAPH_OPT_INTERVAL = fsSettings["depth_map_pose_graph_opt_interval"];

        readOptionalRosParam(n, "use_depth_to_map_pose_graph", USE_DEPTH_TO_MAP_POSE_GRAPH);
        readOptionalRosParam(n, "depth_map_weight", DEPTH_MAP_WEIGHT);
        readOptionalRosParam(n, "depth_map_huber", DEPTH_MAP_HUBER);
        readOptionalRosParam(n, "depth_map_min_edges", DEPTH_MAP_MIN_EDGES);
        readOptionalRosParam(n, "depth_map_max_edges_per_frame", DEPTH_MAP_MAX_EDGES_PER_FRAME);
        readOptionalRosParam(n, "depth_map_neighbor_count", DEPTH_MAP_NEIGHBOR_COUNT);
        readOptionalRosParam(n, "depth_map_max_neighbor_dist", DEPTH_MAP_MAX_NEIGHBOR_DIST);
        readOptionalRosParam(n, "depth_map_plane_max_dist", DEPTH_MAP_PLANE_MAX_DIST);
        readOptionalRosParam(n, "depth_map_min_scale", DEPTH_MAP_MIN_SCALE);
        readOptionalRosParam(n, "depth_map_pose_graph_target_keyframes", DEPTH_MAP_POSE_GRAPH_TARGET_KEYFRAMES);
        readOptionalRosParam(n, "depth_map_pose_graph_opt_interval", DEPTH_MAP_POSE_GRAPH_OPT_INTERVAL);

        if (DEPTH_MAP_NEIGHBOR_COUNT < 3)
            DEPTH_MAP_NEIGHBOR_COUNT = 3;
        if (DEPTH_MAP_MAX_EDGES_PER_FRAME < 1)
            DEPTH_MAP_MAX_EDGES_PER_FRAME = 1;
        if (DEPTH_MAP_MIN_EDGES < 1)
            DEPTH_MAP_MIN_EDGES = 1;
        if (DEPTH_MAP_MAX_NEIGHBOR_DIST <= 0.0)
            DEPTH_MAP_MAX_NEIGHBOR_DIST = 1.0;
        if (DEPTH_MAP_PLANE_MAX_DIST <= 0.0)
            DEPTH_MAP_PLANE_MAX_DIST = 0.2;
        if (DEPTH_MAP_MIN_SCALE < 0.0)
            DEPTH_MAP_MIN_SCALE = 0.0;
        if (DEPTH_MAP_POSE_GRAPH_TARGET_KEYFRAMES < 1)
            DEPTH_MAP_POSE_GRAPH_TARGET_KEYFRAMES = 1;
        if (DEPTH_MAP_POSE_GRAPH_OPT_INTERVAL < 0)
            DEPTH_MAP_POSE_GRAPH_OPT_INTERVAL = 0;

        ROS_INFO("depth-to-map pose graph: %d weight: %.3f huber: %.3f min_edges: %d max_edges: %d neighbors: %d max_neighbor: %.3f plane_max: %.3f min_scale: %.3f target_kfs: %d opt_interval: %d",
                 USE_DEPTH_TO_MAP_POSE_GRAPH, DEPTH_MAP_WEIGHT, DEPTH_MAP_HUBER,
                 DEPTH_MAP_MIN_EDGES, DEPTH_MAP_MAX_EDGES_PER_FRAME,
                 DEPTH_MAP_NEIGHBOR_COUNT, DEPTH_MAP_MAX_NEIGHBOR_DIST,
                 DEPTH_MAP_PLANE_MAX_DIST, DEPTH_MAP_MIN_SCALE,
                 DEPTH_MAP_POSE_GRAPH_TARGET_KEYFRAMES,
                 DEPTH_MAP_POSE_GRAPH_OPT_INTERVAL);
        //OctreePointCloudDensity has no ::Ptr
        posegraph.octree = new pcl::octree::OctreePointCloudDensity<pcl::PointXYZ>(RESOLUTION);
	    posegraph.cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        posegraph.color_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        posegraph.save_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
		posegraph.octree->setInputCloud(posegraph.cloud);
        posegraph.octree->addPointsFromInputCloud();
		// in pcl 1.8.0+, need to set bbox (isVoxelOccupiedAtPoint will check bbox)
		posegraph.octree->defineBoundingBox(-100, -100, -100, 100, 100, 100);
        std::string pkg_path = ros::package::getPath("pose_graph");
        string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
        cout << "vocabulary_file" << vocabulary_file << endl;
        posegraph.loadVocabulary(vocabulary_file);

        BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
        cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());

        fsSettings["image_topic"] >> IMAGE_TOPIC;
        fsSettings["depth_topic"] >> DEPTH_TOPIC;
        std::string image_topic_override;
        if (n.getParam("image_topic", image_topic_override) && !image_topic_override.empty())
        {
            IMAGE_TOPIC = image_topic_override;
            ROS_INFO_STREAM("Override image_topic: " << IMAGE_TOPIC);
        }
        std::string depth_topic_override;
        if (n.getParam("depth_topic", depth_topic_override) && !depth_topic_override.empty())
        {
            DEPTH_TOPIC = depth_topic_override;
            ROS_INFO_STREAM("Override depth_topic: " << DEPTH_TOPIC);
        }
        fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
        fsSettings["output_path"] >> OUTPUT_PATH;
        PCD_OUTPUT_PATH = joinPath(parentPath(OUTPUT_PATH), "pcd");
        posegraph.setVoxbloxOutputDirectory(joinPath(parentPath(OUTPUT_PATH), "voxblox"));
        VINS_RESULT_PATH = OUTPUT_PATH;
        makeDirectoryRecursive(OUTPUT_PATH);
        makeDirectoryRecursive(POSE_GRAPH_SAVE_PATH);
        makeDirectoryRecursive(PCD_OUTPUT_PATH);
        fsSettings["save_image"] >> DEBUG_IMAGE;

        cv::Mat cv_qid, cv_tid;
        fsSettings["extrinsicRotation"] >> cv_qid;
        fsSettings["extrinsicTranslation"] >> cv_tid;
        cv::cv2eigen(cv_qid, qi_d);
        cv::cv2eigen(cv_tid, ti_d);

        VISUALIZE_IMU_FORWARD = fsSettings["visualize_imu_forward"];
        LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
        FAST_RELOCALIZATION = fsSettings["fast_relocalization"];
        VINS_RESULT_PATH = VINS_RESULT_PATH + "/vins_result_loop.csv";
        std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
        fout.close();
        fsSettings.release();
        //not used
        if (LOAD_PREVIOUS_POSE_GRAPH)
        {
            printf("load pose graph\n");
            m_process.lock();
            posegraph.loadPoseGraph();
            posegraph.loadVoxbloxMap();
            m_process.unlock();
            printf("load pose graph finish\n");
            load_flag = 1;
        }
        else
        {
            printf("no previous pose graph\n");
            load_flag = 1;
        }
    }

    fsSettings.release();
    // publish camera pose by imu propagate and odometry (Ps and Rs of curr frame)
    // not important
    ros::Subscriber sub_imu_forward = n.subscribe("/vins_estimator/imu_propagate", 2000, imu_forward_callback);
    // odometry_buf
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 2000, vio_callback);

    //get image msg, store in image_buf
    //ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 2000, image_callback);
    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, IMAGE_TOPIC, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(n, DEPTH_TOPIC, 1);
    //message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(sub_image, sub_depth, 2000);
    // fit fisheye camera
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_image, sub_depth);
    sync.registerCallback(boost::bind(&image_callback, _1, _2));

    //get keyframe_pose(Ps and Rs), store in pose_buf (marginalization_flag == 0)
    ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000, pose_callback);
    //get extrinsic (ric qic  odometry.pose.pose.position and odometry.pose.pose.orientation)
    //update tic and qic real-time
    ros::Subscriber sub_extrinsic = n.subscribe("/vins_estimator/extrinsic", 2000, extrinsic_callback);
    //get keyframe_point(pointclude), store in point_buf (marginalization_flag == 0)
    ros::Subscriber sub_point = n.subscribe("/vins_estimator/keyframe_point", 2000, point_callback);


    // do relocalization here.
    // pose_graph publish match_points to vins_estimator, estimator then publish relo_relative_pose
    ros::Subscriber sub_relo_relative_pose = n.subscribe("/vins_estimator/relo_relative_pose", 2000, relo_relative_pose_callback);

    pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_key_odometrys = n.advertise<visualization_msgs::Marker>("key_odometrys", 1000);
    //not used
    pub_vio_path = n.advertise<nav_msgs::Path>("no_loop_path", 1000);
    pub_match_points = n.advertise<sensor_msgs::PointCloud>("match_points", 100);
    ros::ServiceServer save_map_service = n.advertiseService("save_map", saveMapService);

    std::thread measurement_process;
    std::thread keyboard_command_process;
    // main thread
    measurement_process = std::thread(process);
    // not used
    keyboard_command_process = std::thread(command);
    keyboard_command_process.detach();


    ros::spin();
    if (AUTO_SAVE_MAP_ON_EXIT)
    {
        std::string save_message;
        if (savePoseGraphOutputs(&save_message))
            ROS_INFO_STREAM("Auto-saved map on exit: " << save_message);
        else
            ROS_WARN_STREAM("Auto-save map on exit skipped: " << save_message);
    }
    if (measurement_process.joinable())
        measurement_process.join();

    return 0;
}
