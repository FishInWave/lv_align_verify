#ifndef ZED_UTILS_HPP
#define ZED_UTILS_HPP
#include <sl/Camera.hpp>
#include <thread>
/**
 * @brief 打开zed，最多尝试try_count次，返回错误码
 * 
 * @param zed 
 * @param try_count 默认10次
 */
bool open_zed(sl::Camera &zed, sl::InitParameters init_params, int try_count = 10)
{
    while (try_count--)
    {
        sl::ERROR_CODE err = zed.open(init_params);
        if (err != sl::ERROR_CODE::SUCCESS)
        {
            printf("can not open ZED. %s\n", toString(err).c_str());
            printf("Trying again: %d\n",10-try_count);
            zed.close();
            // 失败的话休眠一秒
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }else{
            return true;
        }
    }
    return false;
}

ros::Time slTime2Ros(sl::Timestamp t)
{
  uint32_t sec = static_cast<uint32_t>(t.getNanoseconds() / 1000000000);
  uint32_t nsec = static_cast<uint32_t>(t.getNanoseconds() % 1000000000);
  return ros::Time(sec, nsec);
}

#endif