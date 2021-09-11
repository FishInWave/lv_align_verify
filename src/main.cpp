#include <lv_align_verify.hpp>

int main(int argc,char** argv){
    ros::init(argc,argv,"lv_align_verify");
    ros::NodeHandle nh("~");
    LVAlignVerify cuc(nh,argc,argv);
    std::thread displaythread(&LVAlignVerify::displayFrameStream,&cuc);
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    displaythread.join();

    return 0;
}