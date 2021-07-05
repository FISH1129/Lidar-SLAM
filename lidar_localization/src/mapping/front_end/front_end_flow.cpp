/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 */
#include "lidar_localization/mapping/front_end/front_end_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h.in"
#include <chrono>

namespace lidar_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic) {
    cloud_sub_ptr_      = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, odom_topic, "/map", "/lidar", 100);

    front_end_ptr_      = std::make_shared<FrontEnd>();
    //pub_bounding_boxs_  = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_bounding_boxs", 5);
}

bool FrontEndFlow::Run() {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        if (UpdateLaserOdometry()) {
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used  = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            std::cout << "里程计update时间= " << time_used.count() << " seconds. " << std::endl;

            PublishData();
        }
    }

    return true;
}

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    return true;
}

bool FrontEndFlow::HasData() {
    return cloud_data_buff_.size() > 0;
}

bool FrontEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();//当前帧点云
    cloud_data_buff_.pop_front();

    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
    static bool odometry_inited = false;
    if (!odometry_inited) {
        odometry_inited = true;
        front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());
        return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);//当前点云，里程计位姿矩阵
    }

    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);

}

bool FrontEndFlow::PublishData() {
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);

    return true;
}
}