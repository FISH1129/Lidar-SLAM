/*
 * @Description: 地图匹配定位算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef LIDAR_LOCALIZATION_MATCHING_MATCHING_HPP_
#define LIDAR_LOCALIZATION_MATCHING_MATCHING_HPP_

#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"

#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"
#include "lidar_localization/models/cloud_filter/box_filter.hpp"
#include <Eigen/Geometry>
#include <ros/package.h>
#include <unistd.h>
namespace lidar_localization
{
class Matching
{
  struct MapCellData
  {
    MapCellData()
    {
      mu = 0.0;
      sigma = 0.0;
      point_cnt = 0;
    }
    float mu;
    float sigma;
    int point_cnt;
  };

  enum InitialType
  {
    FullPose = 0,
    OnlyPosition = 1,
    Nothing
  };

public:
  Matching();

  bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
  bool SetGNSSPose(const Eigen::Matrix4f& init_pose);

  void GetGlobalMap(CloudData::CLOUD_PTR& global_map);
  CloudData::CLOUD_PTR& GetLocalMap();
  CloudData::CLOUD_PTR& GetCurrentScan();
  bool HasInited();
  bool HasNewGlobalMap();
  bool HasNewLocalMap();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  bool InitWithConfig();
  bool InitDataPath(const YAML::Node& config_node);
  bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
  bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr,
                  const YAML::Node& config_node);
  bool InitBoxFilter(const YAML::Node& config_node);
  bool InitInitailType(const YAML::Node& config_node);
  bool InitGridMapResolution(const YAML::Node& config_node);

  bool SetInitPose(const Eigen::Matrix4f& init_pose);
  bool InitGlobalMap();
  bool ResetLocalMap(float x, float y, float z);
  void generateGauss2DMapCells();
  double getInitialYawAngle(CloudData::CLOUD_PTR current_cloud);
  void resetMapRange(const Eigen::Vector3f& center_point);

private:
  std::string map_path_ = "";

  std::shared_ptr<BoxFilter> box_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
  std::shared_ptr<RegistrationInterface> registration_ptr_;

  CloudData::CLOUD_PTR local_map_ptr_;
  CloudData::CLOUD_PTR global_map_ptr_;
  CloudData::CLOUD_PTR current_scan_ptr_;
  Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();
  bool has_inited_ = false;
  bool has_new_global_map_ = false;
  bool has_new_local_map_ = false;

  Eigen::Vector3f local_map_origion_;
  Eigen::Vector3f map_min_xyz_;
  Eigen::Vector3f map_max_xyz_;
  int local_map_width_;
  int local_map_height_;
  double grid_map_resolution_ = 0.8;

  std::vector<std::vector<MapCellData>> map_cell_datas_;
  InitialType init_type_;
};
}  // namespace lidar_localization

#endif