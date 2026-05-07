#include "parameters.h"

std::string IMAGE_TOPIC;
std::string DEPTH_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
int LK_FORWARD_BACKWARD_CHECK;
double LK_MAX_FWD_BWD_ERROR;
double LK_MAX_TRACK_ERROR;
bool PUB_THIS_FRAME;
int PCL_DIST;
int U_BOUNDARY;
int D_BOUNDARY;
int L_BOUNDARY;
int R_BOUNDARY;
float PCL_MIN_DIST;
float PCL_MAX_DIST;

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

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

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
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    LK_FORWARD_BACKWARD_CHECK = 0;
    LK_MAX_FWD_BWD_ERROR = 1.5;
    LK_MAX_TRACK_ERROR = -1.0;
    if (!fsSettings["lk_forward_backward_check"].empty())
        LK_FORWARD_BACKWARD_CHECK = static_cast<int>(fsSettings["lk_forward_backward_check"]);
    if (!fsSettings["lk_max_fwd_bwd_error"].empty())
        LK_MAX_FWD_BWD_ERROR = static_cast<double>(fsSettings["lk_max_fwd_bwd_error"]);
    if (!fsSettings["lk_max_track_error"].empty())
        LK_MAX_TRACK_ERROR = static_cast<double>(fsSettings["lk_max_track_error"]);
    ROS_INFO_STREAM("LK forward-backward check: " << LK_FORWARD_BACKWARD_CHECK
                    << ", max fb error: " << LK_MAX_FWD_BWD_ERROR
                    << ", max track error: " << LK_MAX_TRACK_ERROR);
    PCL_DIST = fsSettings["pcl_dist"];
    U_BOUNDARY = fsSettings["u_boundary"];
    D_BOUNDARY = fsSettings["d_boundary"];
    L_BOUNDARY = fsSettings["l_boundary"];
    R_BOUNDARY = fsSettings["r_boundary"];
    PCL_MIN_DIST = fsSettings["pcl_min_dist"];
    PCL_MAX_DIST = fsSettings["pcl_max_dist"];
    if (PCL_DIST <= 0)
        PCL_DIST = 10;
    if (PCL_MAX_DIST <= PCL_MIN_DIST)
    {
        PCL_MIN_DIST = 0.3;
        PCL_MAX_DIST = 6.0;
    }
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;//shan:What's this?---seems a virtual focal used in rejectWithF.
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();


}
