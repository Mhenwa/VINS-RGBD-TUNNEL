#include "voxblox_mapper.h"

#include <cerrno>
#include <cmath>
#include <sys/stat.h>
#include <sys/types.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>

namespace
{
std::string joinPath(const std::string &base, const std::string &name)
{
    if (base.empty())
        return name;
    if (base.back() == '/')
        return base + name;
    return base + "/" + name;
}

std::string parentPath(const std::string &path)
{
    size_t slash_pos = path.find_last_of('/');
    if (slash_pos == std::string::npos)
        return "";
    if (slash_pos == 0)
        return "/";
    return path.substr(0, slash_pos);
}

bool directoryExists(const std::string &path)
{
    struct stat info;
    return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

bool makeDirectoryRecursive(const std::string &path)
{
    if (path.empty() || directoryExists(path))
        return true;

    std::string parent = parentPath(path);
    if (!parent.empty() && parent != path && !makeDirectoryRecursive(parent))
        return false;

    if (mkdir(path.c_str(), 0755) == 0 || errno == EEXIST)
        return directoryExists(path);

    return false;
}
}

VoxbloxMapper::VoxbloxMapper()
    : enabled_(false),
      initialized_(false),
      world_frame_("world"),
      method_("fast"),
      color_mode_("color"),
      slice_level_(0.5),
      update_esdf_every_n_keyframes_(1),
      update_mesh_every_n_keyframes_(5),
      integrated_keyframes_(0),
      publish_tsdf_pointcloud_(true),
      publish_surface_pointcloud_(true),
      publish_esdf_pointcloud_(true),
      publish_slices_(true),
      publish_esdf_map_(true)
{
}

void VoxbloxMapper::configure(ros::NodeHandle &pose_graph_nh)
{
    ros::NodeHandle nh(pose_graph_nh, "voxblox");
    nh.param("enabled", enabled_, true);
    if (!enabled_)
    {
        ROS_INFO("Voxblox mapping disabled.");
        return;
    }

    int voxels_per_side = 16;
    double voxel_size = 0.05;
    double truncation_distance = 0.20;
    double min_ray_length = 0.3;
    double max_ray_length = 6.0;
    double max_weight = 10000.0;
    double esdf_max_distance = 2.0;
    double esdf_default_distance = 2.0;
    double esdf_min_distance = 0.1;
    double esdf_min_diff = 0.001;
    int integrator_threads = 0;

    nh.param("world_frame", world_frame_, world_frame_);
    nh.param("method", method_, method_);
    nh.param("color_mode", color_mode_, color_mode_);
    nh.param("tsdf_voxel_size", voxel_size, voxel_size);
    nh.param("tsdf_voxels_per_side", voxels_per_side, voxels_per_side);
    nh.param("truncation_distance", truncation_distance, truncation_distance);
    nh.param("min_ray_length_m", min_ray_length, min_ray_length);
    nh.param("max_ray_length_m", max_ray_length, max_ray_length);
    nh.param("max_weight", max_weight, max_weight);
    nh.param("voxel_carving_enabled", tsdf_integrator_config_.voxel_carving_enabled,
             tsdf_integrator_config_.voxel_carving_enabled);
    nh.param("use_const_weight", tsdf_integrator_config_.use_const_weight,
             tsdf_integrator_config_.use_const_weight);
    nh.param("allow_clear", tsdf_integrator_config_.allow_clear,
             tsdf_integrator_config_.allow_clear);
    nh.param("use_weight_dropoff", tsdf_integrator_config_.use_weight_dropoff,
             tsdf_integrator_config_.use_weight_dropoff);
    nh.param("anti_grazing", tsdf_integrator_config_.enable_anti_grazing,
             tsdf_integrator_config_.enable_anti_grazing);
    nh.param("start_voxel_subsampling_factor", tsdf_integrator_config_.start_voxel_subsampling_factor,
             tsdf_integrator_config_.start_voxel_subsampling_factor);
    nh.param("max_consecutive_ray_collisions", tsdf_integrator_config_.max_consecutive_ray_collisions,
             tsdf_integrator_config_.max_consecutive_ray_collisions);
    nh.param("max_integration_time_s", tsdf_integrator_config_.max_integration_time_s,
             tsdf_integrator_config_.max_integration_time_s);
    nh.param("integrator_threads", integrator_threads, integrator_threads);

    nh.param("esdf_max_distance_m", esdf_max_distance, esdf_max_distance);
    nh.param("esdf_default_distance_m", esdf_default_distance, esdf_default_distance);
    nh.param("esdf_min_distance_m", esdf_min_distance, esdf_min_distance);
    nh.param("esdf_min_diff_m", esdf_min_diff, esdf_min_diff);
    nh.param("esdf_full_euclidean_distance", esdf_integrator_config_.full_euclidean_distance,
             esdf_integrator_config_.full_euclidean_distance);
    nh.param("esdf_add_occupied_crust", esdf_integrator_config_.add_occupied_crust,
             esdf_integrator_config_.add_occupied_crust);

    nh.param("mesh_min_weight", mesh_config_.min_weight, mesh_config_.min_weight);
    nh.param("mesh_use_color", mesh_config_.use_color, mesh_config_.use_color);
    nh.param("slice_level", slice_level_, slice_level_);
    nh.param("update_esdf_every_n_keyframes", update_esdf_every_n_keyframes_,
             update_esdf_every_n_keyframes_);
    nh.param("update_mesh_every_n_keyframes", update_mesh_every_n_keyframes_,
             update_mesh_every_n_keyframes_);
    nh.param("publish_tsdf_pointcloud", publish_tsdf_pointcloud_, publish_tsdf_pointcloud_);
    nh.param("publish_surface_pointcloud", publish_surface_pointcloud_, publish_surface_pointcloud_);
    nh.param("publish_esdf_pointcloud", publish_esdf_pointcloud_, publish_esdf_pointcloud_);
    nh.param("publish_slices", publish_slices_, publish_slices_);
    nh.param("publish_esdf_map", publish_esdf_map_, publish_esdf_map_);

    tsdf_config_.tsdf_voxel_size = static_cast<voxblox::FloatingPoint>(voxel_size);
    tsdf_config_.tsdf_voxels_per_side = static_cast<size_t>(voxels_per_side);

    tsdf_integrator_config_.default_truncation_distance = static_cast<float>(truncation_distance);
    tsdf_integrator_config_.min_ray_length_m = static_cast<voxblox::FloatingPoint>(min_ray_length);
    tsdf_integrator_config_.max_ray_length_m = static_cast<voxblox::FloatingPoint>(max_ray_length);
    tsdf_integrator_config_.max_weight = static_cast<float>(max_weight);
    if (integrator_threads > 0)
    {
        tsdf_integrator_config_.integrator_threads = static_cast<size_t>(integrator_threads);
        mesh_config_.integrator_threads = static_cast<size_t>(integrator_threads);
    }

    esdf_config_.esdf_voxel_size = tsdf_config_.tsdf_voxel_size;
    esdf_config_.esdf_voxels_per_side = tsdf_config_.tsdf_voxels_per_side;

    esdf_integrator_config_.max_distance_m = static_cast<voxblox::FloatingPoint>(esdf_max_distance);
    esdf_integrator_config_.default_distance_m = static_cast<voxblox::FloatingPoint>(
        std::max(esdf_default_distance, esdf_max_distance));
    esdf_integrator_config_.min_distance_m = static_cast<voxblox::FloatingPoint>(esdf_min_distance);
    esdf_integrator_config_.min_diff_m = static_cast<voxblox::FloatingPoint>(esdf_min_diff);

    mesh_pub_ = nh.advertise<voxblox_msgs::Mesh>("mesh", 1, true);
    surface_pointcloud_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("surface_pointcloud", 1, true);
    tsdf_pointcloud_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("tsdf_pointcloud", 1, true);
    esdf_pointcloud_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("esdf_pointcloud", 1, true);
    tsdf_slice_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("tsdf_slice", 1, true);
    esdf_slice_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("esdf_slice", 1, true);
    esdf_map_pub_ = nh.advertise<voxblox_msgs::Layer>("esdf_map_out", 1, true);

    ensureInitialized();
    ROS_INFO("Voxblox mapping enabled: voxel_size=%.3f, voxels_per_side=%d, method=%s",
             voxel_size, voxels_per_side, method_.c_str());
}

void VoxbloxMapper::setOutputDirectory(const std::string &output_dir)
{
    output_dir_ = output_dir;
    map_filename_ = joinPath(output_dir_, "map.vxblx");
    mesh_filename_ = joinPath(output_dir_, "mesh.ply");
}

void VoxbloxMapper::ensureInitialized()
{
    if (!enabled_ || initialized_)
        return;

    tsdf_map_.reset(new voxblox::TsdfMap(tsdf_config_));
    esdf_map_.reset(new voxblox::EsdfMap(esdf_config_));
    tsdf_integrator_ = voxblox::TsdfIntegratorFactory::create(
        method_, tsdf_integrator_config_, tsdf_map_->getTsdfLayerPtr());
    esdf_integrator_.reset(new voxblox::EsdfIntegrator(
        esdf_integrator_config_, tsdf_map_->getTsdfLayerPtr(), esdf_map_->getEsdfLayerPtr()));
    mesh_layer_.reset(new voxblox::MeshLayer(tsdf_map_->block_size()));
    mesh_integrator_.reset(new voxblox::MeshIntegrator<voxblox::TsdfVoxel>(
        mesh_config_, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get()));
    initialized_ = true;
}

voxblox::Transformation VoxbloxMapper::makeCameraTransform(const Eigen::Matrix3d &R_w_i,
                                                           const Eigen::Vector3d &P_w_i,
                                                           const Eigen::Matrix3d &R_i_c,
                                                           const Eigen::Vector3d &t_i_c) const
{
    const Eigen::Matrix3d R_w_c = R_w_i * R_i_c;
    const Eigen::Vector3d P_w_c = P_w_i + R_w_i * t_i_c;
    const Eigen::Matrix<voxblox::FloatingPoint, 3, 3> R_w_c_voxblox =
        R_w_c.cast<voxblox::FloatingPoint>();
    const voxblox::Rotation rotation(R_w_c_voxblox);
    const voxblox::Point position = P_w_c.cast<voxblox::FloatingPoint>();
    return voxblox::Transformation(rotation, position);
}

void VoxbloxMapper::pointsToVoxblox(const std::vector<cv::Point3f> &points_C,
                                    const std::vector<cv::Vec3b> &colors_bgr,
                                    voxblox::Pointcloud *points,
                                    voxblox::Colors *colors) const
{
    points->clear();
    colors->clear();
    points->reserve(points_C.size());
    colors->reserve(points_C.size());
    for (size_t i = 0; i < points_C.size(); ++i)
    {
        const cv::Point3f &point = points_C[i];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            continue;
        points->push_back(voxblox::Point(point.x, point.y, point.z));
        if (i < colors_bgr.size())
        {
            const cv::Vec3b &bgr = colors_bgr[i];
            colors->push_back(voxblox::Color(bgr[2], bgr[1], bgr[0]));
        }
        else
        {
            colors->push_back(voxblox::Color::Gray());
        }
    }
}

void VoxbloxMapper::integrateKeyFrame(const std::vector<cv::Point3f> &points_C,
                                      const std::vector<cv::Vec3b> &colors_bgr,
                                      const Eigen::Matrix3d &R_w_i,
                                      const Eigen::Vector3d &P_w_i,
                                      const Eigen::Matrix3d &R_i_c,
                                      const Eigen::Vector3d &t_i_c,
                                      const ros::Time &stamp)
{
    if (!enabled_ || points_C.empty())
        return;
    ensureInitialized();

    voxblox::Pointcloud points;
    voxblox::Colors colors;
    pointsToVoxblox(points_C, colors_bgr, &points, &colors);
    if (points.empty())
        return;

    const voxblox::Transformation T_w_c = makeCameraTransform(R_w_i, P_w_i, R_i_c, t_i_c);
    tsdf_integrator_->integratePointCloud(T_w_c, points, colors, false);
    ++integrated_keyframes_;

    const bool update_esdf = update_esdf_every_n_keyframes_ > 0 &&
                             integrated_keyframes_ % update_esdf_every_n_keyframes_ == 0;
    const bool mesh_empty = mesh_layer_->getNumberOfAllocatedMeshes() == 0;
    const bool update_mesh = update_mesh_every_n_keyframes_ > 0 &&
                             (mesh_empty ||
                              integrated_keyframes_ % update_mesh_every_n_keyframes_ == 0);
    if (update_esdf)
        updateEsdfIncremental();
    if (update_mesh)
        updateMesh(true);

    if (update_esdf || update_mesh)
        publishAll(stamp);
}

void VoxbloxMapper::rebuild(const std::vector<std::vector<cv::Point3f> > &keyframe_points_C,
                            const std::vector<std::vector<cv::Vec3b> > &keyframe_colors_bgr,
                            const std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d> > &poses_w_i,
                            const Eigen::Matrix3d &R_i_c,
                            const Eigen::Vector3d &t_i_c,
                            const ros::Time &stamp)
{
    if (!enabled_ || keyframe_points_C.empty() || keyframe_points_C.size() != poses_w_i.size())
        return;
    ensureInitialized();
    clear();

    size_t integrated_frames = 0;
    size_t integrated_points = 0;
    for (size_t i = 0; i < keyframe_points_C.size(); ++i)
    {
        voxblox::Pointcloud points;
        voxblox::Colors colors;
        const std::vector<cv::Vec3b> empty_colors;
        const std::vector<cv::Vec3b> &colors_bgr =
            i < keyframe_colors_bgr.size() ? keyframe_colors_bgr[i] : empty_colors;
        pointsToVoxblox(keyframe_points_C[i], colors_bgr, &points, &colors);
        if (points.empty())
            continue;

        const voxblox::Transformation T_w_c = makeCameraTransform(
            poses_w_i[i].first, poses_w_i[i].second, R_i_c, t_i_c);
        tsdf_integrator_->integratePointCloud(T_w_c, points, colors, false);
        ++integrated_frames;
        integrated_points += points.size();
    }

    integrated_keyframes_ = static_cast<int>(integrated_frames);
    updateEsdfBatch();
    updateMesh(false);
    publishAll(stamp);
    ROS_INFO("Voxblox rebuild done: %zu keyframes, %zu depth points", integrated_frames, integrated_points);
}

void VoxbloxMapper::clear()
{
    if (!initialized_)
        return;
    tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
    esdf_map_->getEsdfLayerPtr()->removeAllBlocks();
    esdf_integrator_->clear();
    mesh_layer_->clear();
}

void VoxbloxMapper::updateEsdfIncremental()
{
    if (!initialized_ || tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() == 0)
        return;
    esdf_integrator_->updateFromTsdfLayer(true);
}

void VoxbloxMapper::updateEsdfBatch()
{
    if (!initialized_ || tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() == 0)
        return;
    esdf_integrator_->updateFromTsdfLayerBatch();
}

void VoxbloxMapper::updateMesh(bool only_updated_blocks)
{
    if (!initialized_ || tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() == 0)
        return;
    const bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(only_updated_blocks, clear_updated_flag);
}

void VoxbloxMapper::publishAll(const ros::Time &stamp)
{
    if (!enabled_ || !initialized_)
        return;
    publishPointclouds(stamp);
    publishMesh(stamp);
    publishEsdfMap(stamp);
}

void VoxbloxMapper::publishMesh(const ros::Time &stamp)
{
    voxblox_msgs::Mesh mesh_msg;
    voxblox::generateVoxbloxMeshMsg(mesh_layer_, voxblox::getColorModeFromString(color_mode_), &mesh_msg);
    mesh_msg.header.stamp = stamp;
    mesh_msg.header.frame_id = world_frame_;
    mesh_pub_.publish(mesh_msg);
}

void VoxbloxMapper::publishPointclouds(const ros::Time &stamp)
{
    if (publish_surface_pointcloud_ && surface_pointcloud_pub_.getNumSubscribers() > 0)
    {
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
        const double surface_distance_thresh = tsdf_map_->voxel_size() * 0.75;
        voxblox::createSurfacePointcloudFromTsdfLayer(
            tsdf_map_->getTsdfLayer(), surface_distance_thresh, &pointcloud);
        pointcloud.header.frame_id = world_frame_;
        pointcloud.header.stamp = stamp.toNSec() / 1000ull;
        surface_pointcloud_pub_.publish(pointcloud);
    }

    if (publish_tsdf_pointcloud_ && tsdf_pointcloud_pub_.getNumSubscribers() > 0)
    {
        pcl::PointCloud<pcl::PointXYZI> pointcloud;
        voxblox::createDistancePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(), &pointcloud);
        pointcloud.header.frame_id = world_frame_;
        pointcloud.header.stamp = stamp.toNSec() / 1000ull;
        tsdf_pointcloud_pub_.publish(pointcloud);
    }

    if (publish_esdf_pointcloud_ && esdf_pointcloud_pub_.getNumSubscribers() > 0)
    {
        pcl::PointCloud<pcl::PointXYZI> pointcloud;
        voxblox::createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(), &pointcloud);
        pointcloud.header.frame_id = world_frame_;
        pointcloud.header.stamp = stamp.toNSec() / 1000ull;
        esdf_pointcloud_pub_.publish(pointcloud);
    }

    if (publish_slices_ && tsdf_slice_pub_.getNumSubscribers() > 0)
    {
        pcl::PointCloud<pcl::PointXYZI> pointcloud;
        voxblox::createDistancePointcloudFromTsdfLayerSlice(
            tsdf_map_->getTsdfLayer(), 2, static_cast<voxblox::FloatingPoint>(slice_level_), &pointcloud);
        pointcloud.header.frame_id = world_frame_;
        pointcloud.header.stamp = stamp.toNSec() / 1000ull;
        tsdf_slice_pub_.publish(pointcloud);
    }

    if (publish_slices_ && esdf_slice_pub_.getNumSubscribers() > 0)
    {
        pcl::PointCloud<pcl::PointXYZI> pointcloud;
        voxblox::createDistancePointcloudFromEsdfLayerSlice(
            esdf_map_->getEsdfLayer(), 2, static_cast<voxblox::FloatingPoint>(slice_level_), &pointcloud);
        pointcloud.header.frame_id = world_frame_;
        pointcloud.header.stamp = stamp.toNSec() / 1000ull;
        esdf_slice_pub_.publish(pointcloud);
    }
}

void VoxbloxMapper::publishEsdfMap(const ros::Time & /*stamp*/)
{
    if (!publish_esdf_map_)
        return;
    voxblox_msgs::Layer layer_msg;
    voxblox::serializeLayerAsMsg<voxblox::EsdfVoxel>(
        esdf_map_->getEsdfLayer(), false, &layer_msg, voxblox::MapDerializationAction::kReset);
    esdf_map_pub_.publish(layer_msg);
}

bool VoxbloxMapper::ensureOutputDirectory() const
{
    if (output_dir_.empty())
    {
        ROS_WARN("Voxblox output directory is empty.");
        return false;
    }
    if (!makeDirectoryRecursive(output_dir_))
    {
        ROS_ERROR("Failed to create Voxblox output directory: %s", output_dir_.c_str());
        return false;
    }
    return true;
}

bool VoxbloxMapper::saveMap()
{
    if (!enabled_ || !initialized_)
        return false;
    if (!ensureOutputDirectory())
        return false;

    updateEsdfBatch();
    updateMesh(false);
    const bool save_tsdf = voxblox::io::SaveLayer(tsdf_map_->getTsdfLayer(), map_filename_, true);
    const bool save_esdf = voxblox::io::SaveLayer(esdf_map_->getEsdfLayer(), map_filename_, false);
    const bool save_mesh = voxblox::outputMeshLayerAsPly(mesh_filename_, *mesh_layer_);
    if (save_tsdf && save_esdf)
        ROS_INFO("Saved Voxblox map: %s", map_filename_.c_str());
    else
        ROS_ERROR("Failed to save Voxblox map: %s", map_filename_.c_str());
    if (!save_mesh)
        ROS_WARN("Failed to save Voxblox mesh: %s", mesh_filename_.c_str());
    return save_tsdf && save_esdf;
}

bool VoxbloxMapper::loadMap()
{
    if (!enabled_)
        return false;
    ensureInitialized();
    if (map_filename_.empty())
    {
        ROS_WARN("Voxblox map filename is empty.");
        return false;
    }

    clear();
    const bool multiple_layer_support = true;
    const bool load_tsdf = voxblox::io::LoadBlocksFromFile<voxblox::TsdfVoxel>(
        map_filename_, voxblox::Layer<voxblox::TsdfVoxel>::BlockMergingStrategy::kReplace,
        multiple_layer_support, tsdf_map_->getTsdfLayerPtr());
    const bool load_esdf = voxblox::io::LoadBlocksFromFile<voxblox::EsdfVoxel>(
        map_filename_, voxblox::Layer<voxblox::EsdfVoxel>::BlockMergingStrategy::kReplace,
        multiple_layer_support, esdf_map_->getEsdfLayerPtr());

    if (!load_tsdf || !load_esdf)
    {
        ROS_WARN("Could not load Voxblox sidecar map: %s", map_filename_.c_str());
        return false;
    }

    updateMesh(false);
    publishAll(ros::Time::now());
    ROS_INFO("Loaded Voxblox sidecar map: %s", map_filename_.c_str());
    return true;
}
