#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;
int USE_DEPTH_TO_MAP = 0;
int DEPTH_MAP_REBUILD_EACH_ITERATION = 1;
double DEPTH_MAP_WEIGHT = 100.0;
double DEPTH_MAP_HUBER = 1.0;
int DEPTH_MAP_MIN_EDGES = 50;
int DEPTH_MAP_MAX_EDGES_PER_FRAME = 800;
int DEPTH_MAP_NEIGHBOR_COUNT = 5;
double DEPTH_MAP_MAX_NEIGHBOR_DIST = 1.0;
double DEPTH_MAP_PLANE_MAX_DIST = 0.2;
double DEPTH_MAP_MIN_SCALE = 0.1;
int DEPTH_MAP_UNCERTAINTY_ENABLE = 0;
double DEPTH_MAP_UNCERTAINTY_MIN_WEIGHT = 0.20;
double DEPTH_MAP_UNCERTAINTY_MAX_WEIGHT = 1.00;
double DEPTH_MAP_UNCERTAINTY_RANGE = 4.0;
double DEPTH_MAP_UNCERTAINTY_PLANE_SIGMA = 0.05;
double DEPTH_MAP_UNCERTAINTY_RESIDUAL_SIGMA = 0.10;
double DEPTH_CLOUD_SYNC_TOL = 0.02;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

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

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }

    if (!fsSettings["use_depth_to_map"].empty())
        USE_DEPTH_TO_MAP = fsSettings["use_depth_to_map"];
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
    if (!fsSettings["depth_map_uncertainty_enable"].empty())
        DEPTH_MAP_UNCERTAINTY_ENABLE = fsSettings["depth_map_uncertainty_enable"];
    if (!fsSettings["depth_map_uncertainty_min_weight"].empty())
        DEPTH_MAP_UNCERTAINTY_MIN_WEIGHT = fsSettings["depth_map_uncertainty_min_weight"];
    if (!fsSettings["depth_map_uncertainty_max_weight"].empty())
        DEPTH_MAP_UNCERTAINTY_MAX_WEIGHT = fsSettings["depth_map_uncertainty_max_weight"];
    if (!fsSettings["depth_map_uncertainty_range"].empty())
        DEPTH_MAP_UNCERTAINTY_RANGE = fsSettings["depth_map_uncertainty_range"];
    if (!fsSettings["depth_map_uncertainty_plane_sigma"].empty())
        DEPTH_MAP_UNCERTAINTY_PLANE_SIGMA = fsSettings["depth_map_uncertainty_plane_sigma"];
    if (!fsSettings["depth_map_uncertainty_residual_sigma"].empty())
        DEPTH_MAP_UNCERTAINTY_RESIDUAL_SIGMA = fsSettings["depth_map_uncertainty_residual_sigma"];
    if (!fsSettings["depth_map_rebuild_each_iteration"].empty())
        DEPTH_MAP_REBUILD_EACH_ITERATION = fsSettings["depth_map_rebuild_each_iteration"];
    if (!fsSettings["depth_cloud_sync_tol"].empty())
        DEPTH_CLOUD_SYNC_TOL = fsSettings["depth_cloud_sync_tol"];

    readOptionalRosParam(n, "use_depth_to_map", USE_DEPTH_TO_MAP);
    readOptionalRosParam(n, "depth_map_weight", DEPTH_MAP_WEIGHT);
    readOptionalRosParam(n, "depth_map_huber", DEPTH_MAP_HUBER);
    readOptionalRosParam(n, "depth_map_min_edges", DEPTH_MAP_MIN_EDGES);
    readOptionalRosParam(n, "depth_map_max_edges_per_frame", DEPTH_MAP_MAX_EDGES_PER_FRAME);
    readOptionalRosParam(n, "depth_map_neighbor_count", DEPTH_MAP_NEIGHBOR_COUNT);
    readOptionalRosParam(n, "depth_map_max_neighbor_dist", DEPTH_MAP_MAX_NEIGHBOR_DIST);
    readOptionalRosParam(n, "depth_map_plane_max_dist", DEPTH_MAP_PLANE_MAX_DIST);
    readOptionalRosParam(n, "depth_map_min_scale", DEPTH_MAP_MIN_SCALE);
    readOptionalRosParam(n, "depth_map_uncertainty_enable", DEPTH_MAP_UNCERTAINTY_ENABLE);
    readOptionalRosParam(n, "depth_map_uncertainty_min_weight", DEPTH_MAP_UNCERTAINTY_MIN_WEIGHT);
    readOptionalRosParam(n, "depth_map_uncertainty_max_weight", DEPTH_MAP_UNCERTAINTY_MAX_WEIGHT);
    readOptionalRosParam(n, "depth_map_uncertainty_range", DEPTH_MAP_UNCERTAINTY_RANGE);
    readOptionalRosParam(n, "depth_map_uncertainty_plane_sigma", DEPTH_MAP_UNCERTAINTY_PLANE_SIGMA);
    readOptionalRosParam(n, "depth_map_uncertainty_residual_sigma", DEPTH_MAP_UNCERTAINTY_RESIDUAL_SIGMA);
    readOptionalRosParam(n, "depth_cloud_sync_tol", DEPTH_CLOUD_SYNC_TOL);

    // Keep the parameters for compatibility, but do not enable this experiment.
    // Full darkroom1/2/3 tests showed it is not stable enough for the final branch.
    DEPTH_MAP_UNCERTAINTY_ENABLE = 0;

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
    if (DEPTH_MAP_UNCERTAINTY_MIN_WEIGHT < 0.0)
        DEPTH_MAP_UNCERTAINTY_MIN_WEIGHT = 0.0;
    if (DEPTH_MAP_UNCERTAINTY_MAX_WEIGHT < DEPTH_MAP_UNCERTAINTY_MIN_WEIGHT)
        DEPTH_MAP_UNCERTAINTY_MAX_WEIGHT = DEPTH_MAP_UNCERTAINTY_MIN_WEIGHT;
    if (DEPTH_MAP_UNCERTAINTY_RANGE <= 0.0)
        DEPTH_MAP_UNCERTAINTY_RANGE = 4.0;
    if (DEPTH_MAP_UNCERTAINTY_PLANE_SIGMA <= 0.0)
        DEPTH_MAP_UNCERTAINTY_PLANE_SIGMA = 0.05;
    if (DEPTH_MAP_UNCERTAINTY_RESIDUAL_SIGMA <= 0.0)
        DEPTH_MAP_UNCERTAINTY_RESIDUAL_SIGMA = 0.10;

    ROS_INFO("depth-to-map vio: %d weight: %.3f huber: %.3f min_edges: %d max_edges: %d neighbors: %d max_neighbor: %.3f plane_max: %.3f min_scale: %.3f sync_tol: %.3f uncertainty: %d [%.2f, %.2f] range: %.2f plane_sigma: %.3f residual_sigma: %.3f",
             USE_DEPTH_TO_MAP, DEPTH_MAP_WEIGHT, DEPTH_MAP_HUBER,
             DEPTH_MAP_MIN_EDGES, DEPTH_MAP_MAX_EDGES_PER_FRAME,
             DEPTH_MAP_NEIGHBOR_COUNT, DEPTH_MAP_MAX_NEIGHBOR_DIST,
             DEPTH_MAP_PLANE_MAX_DIST, DEPTH_MAP_MIN_SCALE, DEPTH_CLOUD_SYNC_TOL,
             DEPTH_MAP_UNCERTAINTY_ENABLE, DEPTH_MAP_UNCERTAINTY_MIN_WEIGHT,
             DEPTH_MAP_UNCERTAINTY_MAX_WEIGHT, DEPTH_MAP_UNCERTAINTY_RANGE,
             DEPTH_MAP_UNCERTAINTY_PLANE_SIGMA, DEPTH_MAP_UNCERTAINTY_RESIDUAL_SIGMA);
    
    fsSettings.release();
}
