#ifndef ZED_CV_PCL_UTIL_HPP
#define ZED_CV_PCL_UTIL_HPP
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <string>
#include <sstream>

using namespace sl;

// Mapping between MAT_TYPE and CV_TYPE
int getOCVtype(sl::MAT_TYPE type)
{
    int cv_type = -1;
    switch (type)
    {
    case MAT_TYPE::F32_C1:
        cv_type = CV_32FC1;
        break;
    case MAT_TYPE::F32_C2:
        cv_type = CV_32FC2;
        break;
    case MAT_TYPE::F32_C3:
        cv_type = CV_32FC3;
        break;
    case MAT_TYPE::F32_C4:
        cv_type = CV_32FC4;
        break;
    case MAT_TYPE::U8_C1:
        cv_type = CV_8UC1;
        break;
    case MAT_TYPE::U8_C2:
        cv_type = CV_8UC2;
        break;
    case MAT_TYPE::U8_C3:
        cv_type = CV_8UC3;
        break;
    case MAT_TYPE::U8_C4:
        cv_type = CV_8UC4;
        break;
    default:
        break;
    }
    return cv_type;
}

/**
 * @brief 将sl::Mat转化为cv::Mat，且使其共享内存
 * 
 * @param input 
 * @return cv::Mat 
 */
cv::Mat slMat2cvMat(Mat &input)
{
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()),
                   input.getPtr<sl::uchar1>(MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}
/**
 * @brief 将一片区域的像素点转换为点云
 * 
 * @param pointcloud_sl 
 * @param roi 
 * @return pcl::PointCloud<Ppcl::PointXYZRGBA>::Ptr 
 */
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr slPC2pclPC(sl::Mat &pointcloud_sl, cv::Rect2d roi)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointXYZRGBA point;
    for (size_t i = roi.x; i < roi.x + roi.width; i++)
    {
        for (size_t j = roi.y; j < roi.y + roi.height; j++)
        {
            sl::float4 p_value;
            pointcloud_sl.getValue<sl::float4>(i, j, &p_value, MEM::CPU);
            if (std::isnormal(p_value.z))
            {
                point.x = p_value.x;
                point.y = p_value.y;
                point.z = p_value.z;
                point.rgb = p_value[3];
                pc->push_back(point);
            }
        }
    }
    // if (!pc->empty())
    // {
    //     const_pc = pc;
    // }

    return pc;
}
/**
 * @brief 根据点的强度来设置BGRA颜色
 * 
 * @param intensity 
 * @return cv::Vec4b 
 */
inline cv::Vec4b computerPointColorWithIntensity(const int intensity)
{
    //透明度设置为50，依次为BGRA，定义最低的为红色，中间为绿色，最高的为蓝色
    // 在每个色段，值都在171-255之间，确保可观测。
    if (intensity < 85)
    {
        // std::cout << 171+intensity << std::endl;
        return cv::Vec4b(0, 0, 171+intensity, 50);
    }
    else if (intensity < 170){
        // std::cout << 86+intensity << std::endl;
        return cv::Vec4b(0,86+intensity,0,50);
        
    }
    // std::cout << intensity << std::endl;
    return cv::Vec4b(intensity,0,0,50);
}
inline void displayPixel(cv::Mat& image,int u,int v,int margin,cv::Vec4b pixel){
    int max_width = image.cols;
    int max_height = image.rows;
    for(int ou = std::max(0,u-margin);ou <= std::min(max_width,u+margin);ou++){
        for(int ov = std::max(0,v-margin);ov<=std::min(max_height,v+margin);ov++){
            image.at<cv::Vec4b>(cv::Point(ou,ov)) = pixel;
        }
    }
}

inline float pointDistance(pcl::PointXYZRGBA point)
{
    return sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}
/**
 * @brief Get the Strof Time Stamp object
 * 
 * @param time 
 * @return std::string 
 */
inline std::string getStrofTimeStamp(ros::Time time){
    // 只在乎0.01s
    std::stringstream ss;
    ss<<long(time.toSec()*100);
    return ss.str();
}

#endif