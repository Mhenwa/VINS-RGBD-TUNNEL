#pragma once

#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/mesh/mesh_layer.h>

class VoxbloxMapper
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VoxbloxMapper();

    void configure(ros::NodeHandle &pose_graph_nh);
    bool enabled() const { return enabled_; }
    void setOutputDirectory(const std::string &output_dir);

    void integrateKeyFrame(const std::vector<cv::Point3f> &points_C,
                           const std::vector<cv::Vec3b> &colors_bgr,
                           const Eigen::Matrix3d &R_w_i,
                           const Eigen::Vector3d &P_w_i,
                           const Eigen::Matrix3d &R_i_c,
                           const Eigen::Vector3d &t_i_c,
                           const ros::Time &stamp);

    void rebuild(const std::vector<std::vector<cv::Point3f> > &keyframe_points_C,
                 const std::vector<std::vector<cv::Vec3b> > &keyframe_colors_bgr,
                 const std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d> > &poses_w_i,
                 const Eigen::Matrix3d &R_i_c,
                 const Eigen::Vector3d &t_i_c,
                 const ros::Time &stamp);

    bool saveMap();
    bool loadMap();

    void publishAll(const ros::Time &stamp);

private:
    voxblox::Transformation makeCameraTransform(const Eigen::Matrix3d &R_w_i,
                                                const Eigen::Vector3d &P_w_i,
                                                const Eigen::Matrix3d &R_i_c,
                                                const Eigen::Vector3d &t_i_c) const;
    void pointsToVoxblox(const std::vector<cv::Point3f> &points_C,
                         const std::vector<cv::Vec3b> &colors_bgr,
                         voxblox::Pointcloud *points,
                         voxblox::Colors *colors) const;
    void clear();
    void updateEsdfBatch();
    void updateEsdfIncremental();
    void updateMesh(bool only_updated_blocks);
    void publishMesh(const ros::Time &stamp);
    void publishPointclouds(const ros::Time &stamp);
    void publishEsdfMap(const ros::Time &stamp);
    void ensureInitialized();
    bool ensureOutputDirectory() const;

    bool enabled_;
    bool initialized_;
    std::string world_frame_;
    std::string method_;
    std::string color_mode_;
    std::string output_dir_;
    std::string map_filename_;
    std::string mesh_filename_;
    double slice_level_;
    int update_esdf_every_n_keyframes_;
    int update_mesh_every_n_keyframes_;
    int integrated_keyframes_;
    bool publish_tsdf_pointcloud_;
    bool publish_surface_pointcloud_;
    bool publish_esdf_pointcloud_;
    bool publish_slices_;
    bool publish_esdf_map_;

    voxblox::TsdfMap::Config tsdf_config_;
    voxblox::TsdfIntegratorBase::Config tsdf_integrator_config_;
    voxblox::EsdfMap::Config esdf_config_;
    voxblox::EsdfIntegrator::Config esdf_integrator_config_;
    voxblox::MeshIntegratorConfig mesh_config_;

    std::shared_ptr<voxblox::TsdfMap> tsdf_map_;
    std::shared_ptr<voxblox::EsdfMap> esdf_map_;
    voxblox::TsdfIntegratorBase::Ptr tsdf_integrator_;
    std::unique_ptr<voxblox::EsdfIntegrator> esdf_integrator_;
    std::shared_ptr<voxblox::MeshLayer> mesh_layer_;
    std::unique_ptr<voxblox::MeshIntegrator<voxblox::TsdfVoxel> > mesh_integrator_;

    ros::Publisher mesh_pub_;
    ros::Publisher surface_pointcloud_pub_;
    ros::Publisher tsdf_pointcloud_pub_;
    ros::Publisher esdf_pointcloud_pub_;
    ros::Publisher tsdf_slice_pub_;
    ros::Publisher esdf_slice_pub_;
    ros::Publisher esdf_map_pub_;
};
