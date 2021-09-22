// self
#include "lv_align_verify.hpp"
#include "zed_CV_PCL_util.hpp"
#include "zed_utils.hpp"
#include "file_utils.hpp"
#include "time_utils.hpp"

// zed
#include <sl/Camera.hpp>
// glog
#include <glog/logging.h>
// pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
// opencv
#include <opencv2/highgui.hpp>
// boost
#include <boost/thread.hpp>
// STL
#include <thread>
#include <chrono>
// ROS
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>

LVAlignVerify::LVAlignVerify(ros::NodeHandle nh, int agrc, char **argv)
{
    // ROS初始化
    pnh_ = nh;
    // 初始化GLOG
    std::string log_path = ros::package::getPath("lv_align_verify") + "/log";
    FLAGS_log_dir = log_path;
    std::string log_path_name = log_path + "/INFO_";
    google::InitGoogleLogging(argv[0]); // 以程序命名
    google::SetLogDestination(google::GLOG_INFO, log_path_name.c_str());
    google::SetStderrLogging(google::GLOG_INFO);
    google::SetLogFilenameExtension("log_");
    FLAGS_colorlogtostderr = true;          // Set log color
    FLAGS_logbufsecs = 0;                   // Set log output speed(s)
    FLAGS_max_log_size = 1024;              // Set max log file size
    FLAGS_stop_logging_if_full_disk = true; // If disk is full

    initialSystem();
}

void LVAlignVerify::getROSParam()
{
    ROS_INFO("getROSParam");
    pnh_.param<std::string>("camera_resolution", camera_resolution_, "HD1080");
    LOG(INFO) << camera_resolution_;
    pnh_.param<int>("camera_fps", camera_fps_, 30);
    pnh_.param<std::string>("depth_mode", depth_mode_, "ULTRA");
    pnh_.param<float>("depth_minimum", depth_minimum_, 0.3);
    pnh_.param<float>("depth_maximum", depth_maximum_, 20.0);
    pnh_.param<std::string>("coordinate_units", coordinate_units_, "METER");
    pnh_.param<std::string>("coordinate_system", coordinate_system_, "ROS");
    pnh_.param<std::string>("sensing_mode", sensing_mode_, "STANDARD");
    pnh_.param<std::string>("output_directory", output_directory_, "~/Documents/");
    pnh_.param<std::string>("points_topic", points_topic_, "/rslidar_points");
    pnh_.param<bool>("with_RT_field", with_RT_field_, false);
    pnh_.param<bool>("enable_depth", enable_depth_, false);
    pnh_.param<std::vector<double>>("transform_c_l", transform_c_l_V_, std::vector<double>{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1});
    pnh_.param<int>("margin", margin_,0);
    // pnh_.param<double>("c_l_x", c_l_x_,0);
    // pnh_.param<double>("c_l_y", c_l_y_,0);
    // pnh_.param<double>("c_l_z", c_l_z_,0);
    // pnh_.param<double>("c_l_roll", c_l_roll_,0);
    // pnh_.param<double>("c_l_pitch", c_l_pitch_,0);
    // pnh_.param<double>("c_l_yaw", c_l_yaw_,0);
    
    transform_c_l_ = Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(transform_c_l_V_.data(), 4, 4);
}

void LVAlignVerify::initialSystem()
{
    ROS_INFO("initialSystem");
    getROSParam();

    if (camera_resolution_ == "HD1080")
    {
        init_params_.camera_resolution = RESOLUTION::HD1080;
        image_width_ = 1920;
        image_height_ = 1080;
    }
    else if (camera_resolution_ == "HD720")
    {
        init_params_.camera_resolution = RESOLUTION::HD720;
        image_width_ = 1280;
        image_height_ = 720;
    }
    init_params_.camera_fps = camera_fps_;
    LOG(INFO) << "camera resolution: " << camera_resolution_ << "," << image_width_ << "*" << image_height_;
    runtime_parameters_.enable_depth = enable_depth_;
    if (enable_depth_)
    {
        if (depth_mode_ == "ULTRA")
        {
            init_params_.depth_mode = DEPTH_MODE::ULTRA;
        }
        else if (depth_mode_ == "PERFORMANCE")
        {
            init_params_.depth_mode = DEPTH_MODE::ULTRA;
        }
        LOG(INFO) << "depth mode: " << depth_mode_;

        init_params_.depth_maximum_distance = depth_maximum_;
        init_params_.depth_minimum_distance = depth_minimum_;
        LOG(INFO) << "depth range: [" << depth_minimum_ << " , " << depth_maximum_ << " ]";

        if (sensing_mode_ == "STANDARD")
        {
            runtime_parameters_.sensing_mode = SENSING_MODE::STANDARD;
            LOG(INFO) << "depth sensing_mode: " << sensing_mode_;
        }
    }
    else
    {
        LOG(WARNING) << "depth function: closed";
    }

    if (coordinate_units_ == "METER")
    {
        init_params_.coordinate_units = UNIT::METER;
        LOG(INFO) << "Unit: " << coordinate_units_;
    }
    if (coordinate_system_ == "ROS")
    {
        init_params_.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
        transform_c_ros_ << 0, -1, 0, 0,
            0, 0, -1, 0,
            1, 0, 0, 0,
            0, 0, 0, 1;

        LOG(INFO) << "coordinate system: " << coordinate_system_;
    }
    if (!open_zed(zed_, init_params_))
    {
        LOG(WARNING) << "zed cannot opened, please re-plug";
        return;
    }

    key_down_ = ' ';
    cv::namedWindow("lidar-camera align verify", cv::WINDOW_NORMAL);
    LOG(INFO) << "output directory: " << output_directory_;
    pic_output_directory_ = output_directory_ + "/image";
    if (createNewFolder(pic_output_directory_) == -1)
    {
        LOG(WARNING) << "Failed to create the folder: " << pic_output_directory_;
    }
    string y_m_d, h_m_s;
    getLocalTime(y_m_d,h_m_s);
    pic_output_directory_ = pic_output_directory_ + "/" + y_m_d + "-" + h_m_s;
    if (createNewFolder(pic_output_directory_) == -1)
    {
        LOG(WARNING) << "Failed to create the folder: " << pic_output_directory_;
    }
    LOG(INFO) << "picture output directory: " << pic_output_directory_;
    CameraInformation zed_information = zed_.getCameraInformation();
    calibration_params_ = zed_information.camera_configuration.calibration_parameters;
    fx_ = calibration_params_.left_cam.fx;
    fy_ = calibration_params_.left_cam.fy;
    cx_ = calibration_params_.left_cam.cx;
    cy_ = calibration_params_.left_cam.cy;
    K_intrinsic_ << fx_, 0, cx_, 0,
        0, fy_, cy_, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    
    points_sub_ = pnh_.subscribe(points_topic_, 10, &LVAlignVerify::pointsCallback, this);
    // to debug
    points_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("/temp_cloud",10);
    points_pub_2 = pnh_.advertise<sensor_msgs::PointCloud2>("/temp_cloud_2",10);

    LOG(INFO) << "ZED initialization finished successly.";
    cout << "Initial transform from camera to lidar: \n" << transform_c_l_ << endl;
    Eigen::Vector3d eulerAngle = transform_c_l_.block<3,3>(0,0).eulerAngles(2,1,0);
    c_l_roll_ = eulerAngle[0];
    c_l_pitch_ = eulerAngle[1];
    c_l_yaw_ = eulerAngle[2];
    pnh_.setParam("c_l_x",transform_c_l_(0,3));
    pnh_.setParam("c_l_y",transform_c_l_(1,3));
    pnh_.setParam("c_l_z",transform_c_l_(2,3));

    pnh_.setParam("c_l_roll",c_l_roll_);
    pnh_.setParam("c_l_pitch",c_l_pitch_);
    pnh_.setParam("c_l_yaw",c_l_yaw_);
    cout << "Equivalent RPY is: " << eulerAngle.transpose() << endl;
    cout << "Key introduction:\n "
              << "q: quit this program" << std::endl;
    dsrv_ = new dynamic_reconfigure::Server<lv_align_verify::LVAlignVerifyConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<lv_align_verify::LVAlignVerifyConfig>::CallbackType cb = boost::bind(&LVAlignVerify::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}
// 1. 测试延迟：
// 2. 进行投影
void LVAlignVerify::reconfigureCB(lv_align_verify::LVAlignVerifyConfig &config, uint32_t level){
     ROS_INFO("reconfigureCB");
    c_l_x_ = config.c_l_x;
    c_l_y_ = config.c_l_y;
    c_l_z_ = config.c_l_z;
    c_l_roll_ = config.c_l_roll;
    c_l_pitch_ = config.c_l_pitch;
    c_l_yaw_ = config.c_l_yaw;
    transform_mutex_.lock();
    margin_ = config.margin;
    // 我们规定外参符合RPY定义，即XYZ外旋或ZYX内旋，方程始终是
    transform_c_l_.block<3,3>(0,0) = Eigen::AngleAxisd(c_l_yaw_,Eigen::Vector3d(0,0,1)).toRotationMatrix()
    *Eigen::AngleAxisd(c_l_pitch_,Eigen::Vector3d(0,1,0)).toRotationMatrix()
    *Eigen::AngleAxisd(c_l_roll_,Eigen::Vector3d(1,0,0)).toRotationMatrix();
    transform_c_l_.block<3,1>(0,3) = Eigen::Vector3d(c_l_x_,c_l_y_,c_l_z_);
    transform_mutex_.unlock();
}
// 目前版本的实现，激光滞后于相机，但从代码上来看，两者都工作在10Hz。
// 9/22 得到了ZED support的回复，相机的时间戳也是system clock，但是USB传输本身有缓冲区，这个时间戳是图片信息
//      出现在缓冲区里的时刻，因此却是会出现相机明明在激光到达后开始采集，时间戳却反而更早的情况。
// 目前的带宽下时间戳顺序为：
//image_zed.timestamp < lidar_msgs.header.timestamp < pointCallback.begintime < exec grab
// 60FPS下，以image_zed为基准，后续的dt分别为：0.026 0.0296 0.300s，说明该方式下，激光约滞后于相机1.5帧。
void LVAlignVerify::pointsCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    ros::Time t_point_cb = ros::Time::now();
    ros::Time t_lidar = msg->header.stamp;
    Mat image_zed;
    cv::Mat image_cv;
    if (zed_.grab(runtime_parameters_) == ERROR_CODE::SUCCESS)
    {
        ros::Time t_grab = ros::Time::now();
        zed_.retrieveImage(image_zed, VIEW::LEFT);
        ros::Time t_image = slTime2Ros(image_zed.timestamp);
        image_cv = slMat2cvMat(image_zed);
        // 验证时间戳顺序的代码
        vector<double> time_diff;
        time_diff.push_back((t_image-t_lidar).toSec());
        time_diff.push_back((t_point_cb-t_lidar).toSec());
        time_diff.push_back((t_grab-t_lidar).toSec());
        for(double d:time_diff){
            cout << "time diff: " << d << fixed << setprecision(9) << endl;
        }
        cout << endl;
    }
   
    // PointCloud::Ptr cloud(new PointCloud());
    PointCloud::Ptr cloud(new PointCloud());
    PointCloud::Ptr cloud_zed_ros(new PointCloud());
    PointCloud::Ptr cloud_zed_cv(new PointCloud());
    pcl::fromROSMsg(*msg, *cloud);
    transform_mutex_.lock();
    // 转换到ROS坐标系,transform底层已经进行了NaN点删除
    pcl::transformPointCloud(*cloud, *cloud_zed_ros, transform_c_l_);
    transform_mutex_.unlock();
    points_pub_.publish(cloud_zed_ros);
    // 根据IMU信息进行去畸变校正
    deskewPointCloud(cloud_zed_ros, cloud_zed_ros);
    // 点云从ROS坐标系转换到2D CV坐标系中
    Eigen::Matrix4d trans = K_intrinsic_ * transform_c_ros_;
    pcl::transformPointCloud(*cloud_zed_ros, *cloud_zed_cv, trans);
    int c =0;
    // points_pub_2.publish(*cloud_zed_cv);
    for (PointT p : *cloud_zed_cv)
    {
        // 忽略后半部分的激光点，节省算力
        if (p.z <= 0)
        {
            continue;
        }
        int u = p.x / p.z;
        int v = p.y / p.z;
        pixel_ = computerPointColorWithIntensity(p.intensity);
        displayPixel(image_cv,u,v,margin_,pixel_);
        c++;        
    }
    cv::imshow("lidar-camera align verify", image_cv);
    image_stamp_ = msg->header.stamp;
    key_down_ = cv::waitKey(int(1000 / 20));
    processKeyEvent(image_cv);
}


void LVAlignVerify::processKeyEvent(cv::Mat& image)
{
    switch (key_down_)
    {
    case 'q':
    { // 关闭程序
        cv::destroyAllWindows();
        zed_.close();
        ros::shutdown();
        break;
    }
    case 's':
    {
        //保存当前帧
        std::string png_name = pic_output_directory_+"/" + getStrofTimeStamp(image_stamp_)+".jpg";
        cv::imwrite(png_name, image);
        key_down_ = ' ';
        break;
    }

    }
}

void LVAlignVerify::deskewPointCloud(PointCloud::ConstPtr raw_points, PointCloud::ConstPtr deskewed_points)
{
    // TODO 进行IMU验证时，速度不快，没必要进行去畸变校正
    deskewed_points = raw_points;
}


LVAlignVerify::~LVAlignVerify(){
    cout << "The final transform from camera to LiDAR is: \n" << 
    setprecision(6) << transform_c_l_ << endl;
    // 若没有保存文件，则直接删除整个文件夹
    if(isEmptyFolder(pic_output_directory_)){
        removeFolder(pic_output_directory_);
    }
}
/**
 * @brief 从zed的点云图像里获取距离，由于是标定，圈起来的区域比较小，应该可以直接体素滤波。
 * 
 * @param pointcloud_img 
 * @param roi 
 * @return float -1表示错误的距离
 */
bool LVAlignVerify::calculateCentroidOfBbox(Mat &pointcloud_img, cv::Rect2d roi, pcl::PointXYZ &centroid_bbox)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc = slPC2pclPC(pointcloud_img, roi);
    if (pc->empty())
        return false;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setMeanK(5);             // 用于平均距离估计的点数量
    sor.setStddevMulThresh(3.0); // 实际的阈值为 mean+MulTrhresh* stddev,3.0取自正态分布3 sigma
    sor.setInputCloud(pc);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
    sor.filter(*pc_filtered);

    pcl::VoxelGrid<pcl::PointXYZRGBA> vf;
    vf.setLeafSize(0.5, 0.5, 0.5);
    vf.setDownsampleAllData(false);
    vf.setInputCloud(pc_filtered);
    vf.filter(*pc_filtered);

    int p_size = pc_filtered->size();
    if (p_size == 0)
    {
        LOG(INFO) << "There is no point after voxel filter ";
        return false;
    }
    else if (p_size == 1)
    {
        centroid_bbox.getVector3fMap() = pc_filtered->at(0).getVector3fMap();
        LOG(INFO) << "the point: " << centroid_bbox.getArray3fMap();
        return true;
    }
    else
    {
        LOG(INFO) << "There are two points after voxel filter.";
        for (auto p : (*pc_filtered))
        {
            LOG(INFO) << p.getArray3fMap().transpose();
        }
        centroid_bbox.getVector3fMap() = pc_filtered->at(0).getVector3fMap();
        return true;
    }
}