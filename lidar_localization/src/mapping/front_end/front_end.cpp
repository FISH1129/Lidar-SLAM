/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 */
#include "lidar_localization/mapping/front_end/front_end.hpp"

#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/tools/print_info.hpp"
#include "lidar_localization/models/registration/ndt_registration.hpp"
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/cloud_filter/no_filter.hpp"

namespace lidar_localization {
FrontEnd::FrontEnd()
    :local_map_ptr_(new CloudData::CLOUD()) {
    
    InitWithConfig();
}

bool FrontEnd::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/front_end.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------前端初始化-------------------" << std::endl;
    InitParam(config_node);
    InitRegistration(registration_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);

    return true;
}

bool FrontEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_ = config_node["local_frame_num"].as<int>();

    return true;
}

bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "前端选择的点云匹配方式为：" << registration_method << std::endl;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
        LOG(ERROR) << "没找到与 " << registration_method << " 相对应的点云匹配方式!";
        return false;
    }

    return true;
}

bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "前端" << filter_user << "选择的滤波方法为：" << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else if (filter_mothod == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
        return false;
    }

    return true;
}

//原来const CloudData& cloud_data
bool FrontEnd::Update(CloudData & cloud_data, Eigen::Matrix4f & cloud_pose) {
    current_frame_.cloud_data.time = cloud_data.time;
    std::cout <<"原始点云"<< cloud_data.cloud_ptr->points.size() << std::endl;

    std::vector<int> indices_;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices_);
    
    if (!ImportBBOXFromFile()){
        std::cout << "当前帧中没有检测到 运动及潜在运动物体！" << std::endl;
    }

    //CloudData::CLOUD_PTR no_object_cloud_ptr(new CloudData::CLOUD()); //一帧 存放非物体点云

    while(!current_bbox_.empty()){
        CloudBbox current_a_box;
        current_a_box = current_bbox_.front();
        //提取每个bbox点云，放入object_cloud_ptr
      
        if(current_a_box.score > 0.5){
            buff_mutex_.lock();

            pcl::CropBox<CloudData::POINT> region;
            std::vector<int> indices_sub;
            region.setMin(Eigen::Vector4f(-current_a_box.dx/2,	-current_a_box.dy/2,	-current_a_box.dz/2, 1));
	        region.setMax(Eigen::Vector4f( current_a_box.dx/2,	 current_a_box.dy/2,	 current_a_box.dz/2, 1));
            region.setTranslation(Eigen::Vector3f(current_a_box.x, current_a_box.y, current_a_box.z));//前三项平移
	        region.setRotation(Eigen::Vector3f(0, 0, current_a_box.heading));//旋转
            region.setInputCloud(current_frame_.cloud_data.cloud_ptr);
	        region.filter(indices_sub); //bbox里面的点在源点云下的索引
	        indices.insert(indices.end(),indices_sub.begin(),indices_sub.end());
            indices_sub.clear();

            buff_mutex_.unlock();
        }
        current_bbox_.pop_front();
    }
    //提取车身周围范围内的杂点，并将提取到的所有点保存在indices_cluster中
    std::vector<int> indices_cluster;
    pcl::CropBox<pcl::PointXYZ> roof;
    roof.setMin(Eigen::Vector4f(-1.5, -2.1, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setNegative(false); //true是将盒子内的点去除  false是只保留盒子内点云
    roof.setInputCloud(current_frame_.cloud_data.cloud_ptr);
    roof.filter(indices_cluster);
    indices.insert(indices.end(),indices_cluster.begin(),indices_cluster.end());
    indices_cluster.clear();

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices}; //创建一个内点对象，将提取到车身周围点，放到内点对象中
    for (int point : indices)
    {
      inliers->indices.push_back(point);
    }
    
    std::cout <<"原始点云removeNaN后"<< current_frame_.cloud_data.cloud_ptr->points.size() << std::endl;
    pcl::ExtractIndices<CloudData::POINT> extract_out;
    extract_out.setInputCloud(current_frame_.cloud_data.cloud_ptr);
    extract_out.setIndices(inliers);
    extract_out.setNegative(true);  //true提取物体索引之外的点
    extract_out.filter(*current_frame_.cloud_data.cloud_ptr);
    std::cout <<"再去除物体后"<< current_frame_.cloud_data.cloud_ptr->points.size() << std::endl;
    
    std::vector<int>().swap(indices);
    //std::cout<<"清空当前帧物体的indices"<<std::endl;

    //current_frame_.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*no_object_cloud_ptr)); //去除物体
    *cloud_data.cloud_ptr = *current_frame_.cloud_data.cloud_ptr;
    std::cout <<"发布的去除物体后的点"<< cloud_data.cloud_ptr->points.size() << std::endl;

    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);
    std::cout <<"最终去除物体+滤波后"<< filtered_cloud_ptr->points.size() << std::endl;

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    if (local_map_frames_.size() == 0) {
        current_frame_.pose = init_pose_;
        UpdateWithNewFrame(current_frame_);
        cloud_pose = current_frame_.pose;
        std::cout <<"当前帧位姿"<< std::endl;
        std::cout << cloud_pose << std::endl;
        return true;
    }

    // 不是第一帧，就正常匹配
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, current_frame_.pose);
    cloud_pose = current_frame_.pose;

    std::cout <<"当前帧位姿"<< std::endl;
    std::cout << cloud_pose << std::endl;

    // 更新相邻两帧的相对运动
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) {
        UpdateWithNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }

    return true;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
}

bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame) {
    Frame key_frame = new_key_frame;
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    
    // 更新局部地图
    local_map_frames_.push_back(key_frame);
    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CLOUD());
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
                                 *transformed_cloud_ptr, 
                                 local_map_frames_.at(i).pose);

        *local_map_ptr_ += *transformed_cloud_ptr;
    }

    // 更新ndt匹配的目标点云
    // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
    if (local_map_frames_.size() < 10) {
        registration_ptr_->SetInputTarget(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }

    return true;
}

//加载bbox
bool FrontEnd::ImportBBOXFromFile(){
    static int current_index = 1;   //有多少帧 0018-1(原来是4); 0047-2; 0042-1; 0004-3; 0027-1

    CloudBbox init;
    init.x = 0;
    init.y = 0;
    init.z = 0;
    init.dx = 0;
    init.dy = 0;
    init.dz = 0;
    init.heading = 0;
    init.score = 0;

    std::vector<CloudBbox> current_bbox; //当前帧点云的所有bbox

    //ros::package::getPath("lidar_localization") = /home/fbb/SLAM/myslam/MNDT_ground_cluster_4_ws/src/lidar_localization
    std::string current_bbox_file = "/media/fbb/Data/dataset/KITTI_new/bbox/0027/bbox_result/"
                                                        + std::to_string(current_index) +".txt";
    std::string current_bbox_scores_file  = "/media/fbb/Data/dataset/KITTI_new/bbox/0027/scores/"
                                                        + std::to_string(current_index) +".txt";

    std::ifstream origion_read_file;
    origion_read_file.open(current_bbox_file);

    std::ifstream bbox_scores_file;
    bbox_scores_file.open(current_bbox_scores_file);

    if (!bbox_scores_file.is_open() || !origion_read_file.is_open()){
        current_index++;
        return false;
    }

    if (bbox_scores_file.is_open())
    {

        int i = 0; //行计数器
        std::string lidae_data;
        while (getline(bbox_scores_file, lidae_data)) //逐行读取数据并存于lidae_data中，直至数据全部读取
        {
            current_bbox.push_back(init);
            std::istringstream ss(lidae_data);
            float word_data = 0.0;

            while (ss >> word_data){
                current_bbox[i].score = word_data;
            }
            i++;
        }
    }

    if (origion_read_file.is_open())
    {
        int i = 0; //行计数器
        std::string lidae_data;
        while (getline(origion_read_file, lidae_data, '\n')) //逐行读取数据并存于lidae_data中，直至数据全部读取
        {
          std::istringstream ss(lidae_data);
          float word_data;
          int j = 0;
          while (ss >> word_data)
          {
            if (j == 0){
              current_bbox[i].x = word_data;
            }
            else if (j == 1){
              current_bbox[i].y = word_data;
            }
            else if (j == 2){
              current_bbox[i].z = word_data;
            }
            else if (j == 3){
              current_bbox[i].dx = word_data;
            }
            else if (j == 4){
              current_bbox[i].dy = word_data;
            }
            else if (j == 5){
              current_bbox[i].dz = word_data;
            }
            else if (j == 6){
              current_bbox[i].heading = word_data;
            }
            j++;
          }
          current_bbox_.push_back(current_bbox[i]);
          i++;
        }
        std::vector<CloudBbox>().swap(current_bbox);
        current_index++;
        return true;
    }
    std::vector<CloudBbox>().swap(current_bbox);
    current_index++;
    return false;
}

}