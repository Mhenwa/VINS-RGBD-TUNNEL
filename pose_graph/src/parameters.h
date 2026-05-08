#pragma once

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

extern camodocal::CameraPtr m_camera;
extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;
extern Eigen::Matrix<double, 3, 1> ti_d;
extern Eigen::Matrix<double, 3, 3> qi_d;
extern ros::Publisher pub_match_img;
extern ros::Publisher pub_match_points;
extern int VISUALIZATION_SHIFT_X;
extern int VISUALIZATION_SHIFT_Y;
extern std::string BRIEF_PATTERN_FILE;
extern std::string POSE_GRAPH_SAVE_PATH;
extern int ROW;
extern int COL;
extern std::string VINS_RESULT_PATH;
extern int DEBUG_IMAGE;
extern int FAST_RELOCALIZATION;
extern int USE_STRUCTURAL_PLANES;
extern int STRUCT_PLANE_MIN_INLIERS;
extern double STRUCT_PLANE_DISTANCE_THRESHOLD;
extern double STRUCT_PLANE_NORMAL_MERGE_DEG;
extern double STRUCT_PLANE_DISTANCE_MERGE;
extern double STRUCT_PLANE_WEIGHT;
extern double STRUCT_PLANE_HUBER;
extern int STRUCT_PLANE_MAX_PLANES_PER_KEYFRAME;
extern int STRUCT_PLANE_ENABLE_GROUND;
extern int STRUCT_PLANE_ENABLE_WALLS;
extern int LOOP_GEOM_VERIFY;
extern double LOOP_MIN_BOW_SCORE;
extern double LOOP_CANDIDATE_SCORE;
extern int LOOP_ENABLE_FUNDAMENTAL_CHECK;
extern double LOOP_FUNDAMENTAL_THRESHOLD_PX;
extern int LOOP_MIN_PNP_INLIERS;
extern double LOOP_MIN_INLIER_RATIO;
extern double LOOP_MAX_YAW_DEG;
extern double LOOP_MAX_TRANSLATION_M;
extern int LOOP_TEASER_ENABLE;
extern double LOOP_TEASER_NOISE_BOUND;
extern int LOOP_TEASER_MIN_INLIERS;
extern double LOOP_TEASER_MAX_RMSE;
