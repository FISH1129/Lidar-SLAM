/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 */
#include "lidar_localization/matching/matching.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/models/registration/ndt_registration.hpp"
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/cloud_filter/no_filter.hpp"

namespace lidar_localization
{
Matching::Matching()
  : local_map_ptr_(new CloudData::CLOUD())
  , global_map_ptr_(new CloudData::CLOUD())
  , current_scan_ptr_(new CloudData::CLOUD())
{
  InitWithConfig();

  InitGlobalMap();

  ResetLocalMap(0.0, 0.0, 0.0);
}

bool Matching::InitWithConfig()
{
  std::string config_file_path = WORK_SPACE_PATH + "/config/matching/matching.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);
  std::cout << "-----------------地图定位初始化-------------------" << std::endl;
  InitDataPath(config_node);
  InitRegistration(registration_ptr_, config_node);
  InitFilter("global_map", global_map_filter_ptr_, config_node);
  InitFilter("local_map", local_map_filter_ptr_, config_node);
  InitFilter("frame", frame_filter_ptr_, config_node);
  InitBoxFilter(config_node);
  InitInitailType(config_node);
  InitGridMapResolution(config_node);

  return true;
}

bool Matching::InitDataPath(const YAML::Node& config_node)
{
  map_path_ = config_node["map_path"].as<std::string>();
  return true;
}

bool Matching::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node)
{
  std::string registration_method = config_node["registration_method"].as<std::string>();
  std::cout << "地图匹配选择的点云匹配方式为：" << registration_method << std::endl;

  if (registration_method == "NDT")
  {
    registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
  }
  else
  {
    LOG(ERROR) << "没找到与 " << registration_method << " 相对应的点云匹配方式!";
    return false;
  }

  return true;
}

bool Matching::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr,
                          const YAML::Node& config_node)
{
  std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
  std::cout << "地图匹配" << filter_user << "选择的滤波方法为：" << filter_mothod << std::endl;

  if (filter_mothod == "voxel_filter")
  {
    filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
  }
  else if (filter_mothod == "no_filter")
  {
    filter_ptr = std::make_shared<NoFilter>();
  }
  else
  {
    LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
    return false;
  }

  return true;
}

bool Matching::InitBoxFilter(const YAML::Node& config_node)
{
  box_filter_ptr_ = std::make_shared<BoxFilter>(config_node);
  return true;
}

bool Matching::InitInitailType(const YAML::Node& config_node)
{
  if (config_node["init_type"])
  {
    std::string init_type_str = config_node["init_type"].as<std::string>();
    if (init_type_str == "FullPose")
    {
      init_type_ = InitialType::FullPose;
      std::cout << "地图定位初始化的方式为： " << init_type_str << ", 使用GPS的 位置 + 姿态 进行初始化" << std::endl
                << std::endl;
    }
    else if (init_type_str == "OnlyPosition")
    {
      init_type_ = InitialType::OnlyPosition;
      std::cout << "地图定位初始化的方式为： " << init_type_str << ", 仅使用GPS的 位置 进行初始化" << std::endl
                << std::endl;
    }
    else
    {
      std::cout << init_type_str << " 还没研发出来，默认使用FullPose的" << std::endl << std::endl;
      init_type_ = InitialType::FullPose;
    }
  }
  else
  {
    std::cout << "没有设置初始化方法参数，默认使用FullPose的" << std::endl << std::endl;
    init_type_ = InitialType::FullPose;
  }

  return true;
}

bool Matching::InitGridMapResolution(const YAML::Node& config_node)
{
  if (config_node["grid_resolution"])
  {
    grid_map_resolution_ = config_node["grid_resolution"].as<double>();
    grid_map_resolution_ = std::max(0.1, grid_map_resolution_);
  }
  else
  {
    std::cout << "没有设置grid_map_resolution参数，默认使用0.8" << std::endl << std::endl;
    grid_map_resolution_ = 0.8;
  }
  return true;
}

bool Matching::InitGlobalMap()
{
  if (access(map_path_.c_str(), F_OK) != 0)
  {
    std::cout << map_path_ << " 不存在加载工程包下的点云地图文件" << std::endl;
    map_path_ = ros::package::getPath("lidar_localization") + "/localization_data/filtered_map.pcd";
  }
  pcl::io::loadPCDFile(map_path_, *global_map_ptr_);
  LOG(INFO) << "load global map size:" << global_map_ptr_->points.size();

  local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
  LOG(INFO) << "filtered global map size:" << global_map_ptr_->points.size();

  has_new_global_map_ = true;

  return true;
}

bool Matching::ResetLocalMap(float x, float y, float z)
{
  std::vector<float> origin = { x, y, z };
  local_map_origion_ = Eigen::Vector3f(x, y, z);
  box_filter_ptr_->SetOrigin(origin);
  box_filter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

  registration_ptr_->SetInputTarget(local_map_ptr_);

  has_new_local_map_ = true;

  std::vector<float> edge = box_filter_ptr_->GetEdge();
  LOG(INFO) << "new local map:" << edge.at(0) << "," << edge.at(1) << "," << edge.at(2) << "," << edge.at(3) << ","
            << edge.at(4) << "," << edge.at(5) << std::endl
            << std::endl;

  return true;
}

bool Matching::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose)
{
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, indices);

  CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
  frame_filter_ptr_->Filter(cloud_data.cloud_ptr, filtered_cloud_ptr);

  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;

  if (!has_inited_)
  {
    if (init_type_ == InitialType::FullPose)
      predict_pose = current_gnss_pose_;
    else if (init_type_ == InitialType::OnlyPosition)
    {
      static double last_max_prob_yaw_angle = 1000;
      double max_prob_yaw_angle = getInitialYawAngle(cloud_data.cloud_ptr);
      if (last_max_prob_yaw_angle > 100)
      {
        last_max_prob_yaw_angle = max_prob_yaw_angle;
        predict_pose.setIdentity();

        std::cout << " ============= max_prob_yaw_angle is: " << max_prob_yaw_angle << std::endl;
        Eigen::AngleAxisf rotation(max_prob_yaw_angle, Eigen::Vector3f::UnitZ());
        std::cout << "   max_prob_yaw_angle rotation is: " << std::endl << rotation.matrix() << std::endl << std::endl;
        std::cout << "   current_gnss rotation is: " << std::endl
                  << current_gnss_pose_.block<3, 3>(0, 0) << std::endl
                  << std::endl
                  << std::endl;
        predict_pose.block<3, 3>(0, 0) = rotation.matrix();
        predict_pose.block<3, 1>(0, 3) = current_gnss_pose_.block<3, 1>(0, 3);
      }
      else
      {
        if (fabs(last_max_prob_yaw_angle - max_prob_yaw_angle) < 0.03)
        {
          last_max_prob_yaw_angle = max_prob_yaw_angle;

          predict_pose.setIdentity();

          std::cout << " ============= max_prob_yaw_angle is: " << max_prob_yaw_angle << std::endl;
          Eigen::AngleAxisf rotation(max_prob_yaw_angle, Eigen::Vector3f::UnitZ());
          std::cout << "   max_prob_yaw_angle rotation is: " << std::endl
                    << rotation.matrix() << std::endl
                    << std::endl;
          std::cout << "   current_gnss rotation is: " << std::endl
                    << current_gnss_pose_.block<3, 3>(0, 0) << std::endl
                    << std::endl
                    << std::endl;
          predict_pose.block<3, 3>(0, 0) = rotation.matrix();
          predict_pose.block<3, 1>(0, 3) = current_gnss_pose_.block<3, 1>(0, 3);
        }
      }
    }
  }

  // 与地图匹配
  CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
  registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, cloud_pose);
  pcl::transformPointCloud(*cloud_data.cloud_ptr, *current_scan_ptr_, cloud_pose);

  // 更新相邻两帧的相对运动
  step_pose = last_pose.inverse() * cloud_pose;
  predict_pose = cloud_pose * step_pose;
  last_pose = cloud_pose;

  // 匹配之后判断是否需要更新局部地图
  std::vector<float> edge = box_filter_ptr_->GetEdge();
  for (int i = 0; i < 3; i++)
  {
    if (fabs(cloud_pose(i, 3) - edge.at(2 * i)) > 50.0 && fabs(cloud_pose(i, 3) - edge.at(2 * i + 1)) > 50.0)
      continue;
    ResetLocalMap(cloud_pose(0, 3), cloud_pose(1, 3), cloud_pose(2, 3));
    break;
  }

  return true;
}

double Matching::getInitialYawAngle(CloudData::CLOUD_PTR current_cloud)
{
  int angle_size = 270;
  float delta_angle = 2 * M_PI / angle_size;
  float max_prob = -std::numeric_limits<float>::max();
  std::vector<double> probs(angle_size, 0);
  double probs_sum = 0;
  for (int i = 0; i < angle_size; i++)
  {
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    rotation.block<3, 3>(0, 0) = Eigen::AngleAxisf(delta_angle * i, Eigen::Vector3f::UnitZ()).matrix();

    CloudData::CLOUD_PTR transformed_cloud(new CloudData::CLOUD);
    pcl::transformPointCloud(*current_cloud, *transformed_cloud, rotation);
    for (size_t j = 0; j < transformed_cloud->points.size(); j++)
    {
      auto point = transformed_cloud->points[j];
      if (!pcl::isFinite(point))
        continue;
      int cell_x = std::round((point.x - map_min_xyz_(0)) / grid_map_resolution_);
      int cell_y = std::round((point.y - map_min_xyz_(1)) / grid_map_resolution_);
      if (cell_x < 0 || cell_y < 0 || cell_x >= local_map_width_ || cell_y >= local_map_height_)
        continue;
      if (map_cell_datas_[cell_x][cell_y].point_cnt == 0)
        continue;
      float mu = map_cell_datas_[cell_x][cell_y].mu;
      float sigma = map_cell_datas_[cell_x][cell_y].sigma;
      probs[i] += std::exp(-std::pow(point.z - mu, 2) / (2 * sigma));
    }
    probs_sum += probs[i];
  }
  double max_prob_angle = 0;
  for (size_t it = 0; it < probs.size(); it++)
  {
    if (probs[it] > max_prob)
    {
      max_prob = probs[it];
      max_prob_angle = it * delta_angle;
    }
  }
  return max_prob_angle;
}

bool Matching::SetGNSSPose(const Eigen::Matrix4f& gnss_pose)
{
  current_gnss_pose_ = gnss_pose;

  static int gnss_cnt = 0;
  if (gnss_cnt == 0)
  {
    SetInitPose(gnss_pose);
  }
  else if (gnss_cnt > 2)
  {
    has_inited_ = true;
  }
  gnss_cnt++;
  return true;
}

bool Matching::SetInitPose(const Eigen::Matrix4f& init_pose)
{
  init_pose_ = init_pose;
  ResetLocalMap(init_pose(0, 3), init_pose(1, 3), init_pose(2, 3));
  map_min_xyz_ = Eigen::Vector3f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max());
  map_max_xyz_ = Eigen::Vector3f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                                 -std::numeric_limits<float>::max());

  if (init_type_ == InitialType::OnlyPosition)
  {
    generateGauss2DMapCells();
  }

  return true;
}

void Matching::generateGauss2DMapCells()
{
  std::vector<Eigen::Vector3f> center_map_points;
  for (size_t i = 0; i < local_map_ptr_->points.size(); i++)
  {
    auto point = local_map_ptr_->points[i];
    if (!pcl::isFinite(point))
      continue;
    Eigen::Vector3f point_center = Eigen::Vector3f(point.x, point.y, point.z) - local_map_origion_;
    center_map_points.emplace_back(point_center);
    resetMapRange(point_center);
  }

  local_map_width_ = std::round((map_max_xyz_(0) - map_min_xyz_(0)) / grid_map_resolution_);
  local_map_height_ = std::round((map_max_xyz_(1) - map_min_xyz_(1)) / grid_map_resolution_);

  map_cell_datas_.clear();
  map_cell_datas_ = std::vector<std::vector<MapCellData>>(local_map_width_, std::vector<MapCellData>());
  for (int i = 0; i < local_map_width_; i++)
  {
    map_cell_datas_[i].resize(local_map_height_);
  }

  for (size_t i = 0; i < center_map_points.size(); ++i)
  {
    auto pt = center_map_points[i];
    int cell_x = round((pt(0) - map_min_xyz_(0)) / grid_map_resolution_);
    int cell_y = round((pt(1) - map_min_xyz_(1)) / grid_map_resolution_);
    if (cell_x < 0 || cell_y < 0 || cell_x >= local_map_width_ || cell_y >= local_map_height_)
      continue;
    float mu = map_cell_datas_[cell_x][cell_y].mu;
    float sigma = map_cell_datas_[cell_x][cell_y].sigma;
    int point_cnt = map_cell_datas_[cell_x][cell_y].point_cnt;
    if (point_cnt == 0)
    {
      map_cell_datas_[cell_x][cell_y].mu = pt(2);
      map_cell_datas_[cell_x][cell_y].sigma = 0;
      map_cell_datas_[cell_x][cell_y].point_cnt = 1;
    }
    else
    {
      map_cell_datas_[cell_x][cell_y].mu = (point_cnt * mu + pt(2)) / (point_cnt + 1);
      map_cell_datas_[cell_x][cell_y].sigma =
          (point_cnt - 1) * sigma + std::pow(pt(2) - mu, 2) +
          (point_cnt + 1) * std::pow(map_cell_datas_[cell_x][cell_y].mu - mu, 2) +
          2 * (map_cell_datas_[cell_x][cell_y].mu - mu) * (pt(2) - map_cell_datas_[cell_x][cell_y].mu);
      map_cell_datas_[cell_x][cell_y].sigma /= point_cnt;
      map_cell_datas_[cell_x][cell_y].point_cnt++;
    }
  }
}

void Matching::resetMapRange(const Eigen::Vector3f& point_center)
{
  if (point_center(0) < map_min_xyz_(0))
  {
    map_min_xyz_(0) = point_center(0);
  }
  if (point_center(1) < map_min_xyz_(1))
  {
    map_min_xyz_(1) = point_center(1);
  }
  if (point_center(2) < map_min_xyz_(2))
  {
    map_min_xyz_(2) = point_center(2);
  }

  if (point_center(0) > map_max_xyz_(0))
  {
    map_max_xyz_(0) = point_center(0);
  }
  if (point_center(1) > map_max_xyz_(1))
  {
    map_max_xyz_(1) = point_center(1);
  }
  if (point_center(2) > map_max_xyz_(2))
  {
    map_max_xyz_(2) = point_center(2);
  }
}

void Matching::GetGlobalMap(CloudData::CLOUD_PTR& global_map)
{
  global_map_filter_ptr_->Filter(global_map_ptr_, global_map);
  has_new_global_map_ = false;
}

CloudData::CLOUD_PTR& Matching::GetLocalMap()
{
  return local_map_ptr_;
}

CloudData::CLOUD_PTR& Matching::GetCurrentScan()
{
  return current_scan_ptr_;
}

bool Matching::HasInited()
{
  return has_inited_;
}

bool Matching::HasNewGlobalMap()
{
  return has_new_global_map_;
}

bool Matching::HasNewLocalMap()
{
  return has_new_local_map_;
}
}  // namespace lidar_localization