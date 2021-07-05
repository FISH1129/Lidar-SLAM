/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_HPP_
#define LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_HPP_

#include <deque>
#include <mutex>
#include <map>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <string>
#include <sstream>

#include "lidar_localization/sensor_data/cloud_bbox.hpp"

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

#include "lidar_localization/models/registration/ndt_registration.hpp"
#include "lidar_localization/models/registration/icp_registration.hpp"
#include "lidar_localization/models/registration/icp_registration_manual.hpp"
#include "lidar_localization/models/registration/ndt_registration_manual.hpp"

#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/cloud_filter/no_filter.hpp"

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "lidar_localization/models/registration/ndt_registration_manual/togetmedian.hpp"

#include <ros/package.h>
#include "pcl/kdtree/impl/kdtree_flann.hpp"

#include <chrono>
#include <pcl/filters/statistical_outlier_removal.h>  //滤波相关

namespace lidar_localization {
class FrontEnd {
  public:
    struct Frame { 
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };

  public:
    FrontEnd();

    bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
    bool SetInitPose(const Eigen::Matrix4f& init_pose);

  private:
    bool InitWithConfig();
    bool ImportBBOXFromFile();//加载bbox.txt
    bool InitParam(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    bool UpdateWithNewFrame(const Frame& new_key_frame, 
                            const CloudData::CLOUD_PTR& new_center_bbox_ptr,
                            const std::deque<CloudBbox>& new_current_bbox);

  private:
    std::string data_path_ = "";

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<RegistrationInterface> registration_ptr_; 

    std::deque<Frame> local_map_frames_;
    std::deque<CloudData::CLOUD_PTR> local_map_bboxs_;
    std::deque<CloudBbox> current_bbox_; //当前帧点云的bbox

    std::deque<std::deque<CloudBbox>> describe_lm_; //局部地图所有bbox的描述
    std::deque<CloudBbox> describe_lm; //局部地图所有bbox的描述
    std::deque<CloudBbox> describe_current; //当前帧点云的bbox  得分>0.5

    std::vector<int> indices; //保存当前帧所有 物体检测点 的索引
    
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR current_bbox_center_ptr_;//保存当前帧高分bbox的中心点
    CloudData::CLOUD_PTR local_map_bbox_center_ptr_;
    Frame current_frame_;

    std::map<int, std::vector<int>> bbox_points_idx;//添加的
    pcl::KdTreeFLANN<CloudData::POINT> local_map_center_ptr;  //子图bbox中心点

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    float key_frame_distance_ = 2.0;
    int local_frame_num_ = 20;

    int bbox_id = 0; //每帧有多少个bbox
    int v_0 = 10;   //t分布自由度
    int K_s = 1;
    double radius = 3.3;
    double cigma_D; //方差
    double d_median; //中值
    std::vector<double> d;        //变换后点与最近点的距离

    std::mutex buff_mutex_;
};
}

#endif