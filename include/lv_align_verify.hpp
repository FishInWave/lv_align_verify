#ifndef LV_ALIGN_VERIFY_HPP
#define LV_ALIGN_VERIFY_HPP
// third party
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include "lv_align_verify/LVAlignVerifyConfig.h"
// STL
#include <mutex>
#include <atomic>
#include <fstream>
// self
#include "rslidar_utils.hpp"
using namespace sl;
// using PointT = pcl::PointXYZ;
using PointT = RsPointXYZIRT;
using PointCloud = pcl::PointCloud<PointT>;
// 前为zed，后为uwb

class LVAlignVerify
{
private:
    // ros  
    ros::Subscriber points_sub_;
    ros::Publisher points_pub_;
    ros::Publisher points_pub_2;
    ros::NodeHandle pnh_;
        // ros 动态调参 
    dynamic_reconfigure::Server<lv_align_verify::LVAlignVerifyConfig> *dsrv_;
    int margin_; // 激光点的margin，值越大越粗
    double c_l_x_,c_l_y_,c_l_z_,c_l_roll_,c_l_pitch_,c_l_yaw_;
    // zed intrinsic
    Camera zed_;
    InitParameters init_params_;
    CalibrationParameters calibration_params_;
    RuntimeParameters runtime_parameters_;
    Eigen::Matrix4d K_intrinsic_;
    float k1_,k2_,k3_,p1_,p2_; // 去畸变参数，可能用不上
    float fx_,fy_,cx_,cy_; // 内参系数
    // zed config
    std::string camera_resolution_;
    int image_width_,image_height_;// x and y
    int camera_fps_;
    bool enable_depth_;
    std::string depth_mode_;
    float depth_minimum_;
    float depth_maximum_;
    std::string coordinate_units_;
    std::string coordinate_system_;
    std::string sensing_mode_;
    // lidar config
    std::string points_topic_;
    bool with_RT_field_;
    // flag
    char key_down_; // 按下的按键
    // global
    std::string output_directory_;
    std::string pic_output_directory_;
    std::ofstream transformation_ofs_;
    double z_camera_to_uwb_;
    Eigen::Matrix4d transform_c_ros_; // c_cv to c_ros 因为IMU的坐标系是符合ROS的 
    std::vector<double> transform_c_l_V_; // 行排列的向量
    Eigen::Matrix4d transform_c_l_; // 外参矩阵
    cv::Vec4b pixel_; // 激光点的颜色定义
    std::mutex transform_mutex_;
    ros::Time image_stamp_; // 图片时间戳

public:
    LVAlignVerify(ros::NodeHandle nh, int argc, char **argv);
    LVAlignVerify() = default;
    ~LVAlignVerify();
    /**
     * @brief 初始化需要用到的数组等
     * 
     */
    void initialSystem();

    void getROSParam();
    // 接收激光雷达点云后，将其转换到zed2_left_camera_frame,进行图像采集。
    void pointsCallback(const sensor_msgs::PointCloud2ConstPtr msg);

    // 去畸变校正
    // 旋转中心直接使用左目吧，省去一些操作。
    void deskewPointCloud(PointCloud::ConstPtr raw_points,PointCloud::ConstPtr deskewed_points);
    /**
     * @brief 处理按键
     * 
     */
    void processKeyEvent(cv::Mat& image);
    void reconfigureCB(lv_align_verify::LVAlignVerifyConfig &config, uint32_t level);

    //TODO 未被使用的函数，下次迭代时，考虑删除
    //计算2D BBOX质心
    bool calculateCentroidOfBbox(Mat &pointcloud_img, cv::Rect2d roi,pcl::PointXYZ& centroid_bbox);

    void displayFrameStream(){}

    void projectCloudIntoImage(const PointCloud::ConstPtr pointcloud, const cv::Mat image){}
};

#endif