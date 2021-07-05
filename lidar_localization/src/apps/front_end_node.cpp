/*
 * @Description: 前端里程计的node文件
 */
#include <ros/ros.h>
#include "glog/logging.h"
#include <chrono>

#include "lidar_localization/global_defination/global_defination.h.in"
#include "lidar_localization/mapping/front_end/front_end_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    std::string cloud_topic, odom_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");//若前端匹配订阅的是没有地面的点云："/points_noground_synced"
    nh.param<std::string>("odom_topic", odom_topic,   "/laser_odom");

    std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh, cloud_topic, odom_topic);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        //std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        front_end_flow_ptr->Run();
        //std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        //std::chrono::duration<double> time_used  = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        //std::cout << "前端里程计消耗时间= " << time_used.count() << " seconds. " << std::endl;
        rate.sleep();
    }

    return 0;
}