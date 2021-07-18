/*
 * @Description: ICP 匹配模块
 * @Author: Zm Liu
 * @Date:
 */
#include "lidar_localization/models/registration/icp_registration.hpp"

#include "glog/logging.h"

namespace lidar_localization
{
ICPRegistration::ICPRegistration(const YAML::Node& node)
  : icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT,CloudData::POINT>())
{
  float euclid_eps         = node["euclid_eps"].as<float>();
  float max_correspond_dis = node["max_correspond_dis"].as<float>();
  float trans_eps          = node["trans_eps"].as<float>();
  int max_iter             = node["max_iter"].as<int>();
  SetRegistrationParam(euclid_eps, max_correspond_dis, trans_eps, max_iter);
}

ICPRegistration::ICPRegistration(float euclid_eps, float max_correspond_dis, float trans_eps, int max_iter)
  : icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT,CloudData::POINT>())
{
  SetRegistrationParam(euclid_eps, max_correspond_dis, trans_eps, max_iter);
}

bool ICPRegistration::SetRegistrationParam(float euclid_eps, float max_correspond_dis, float trans_eps, int max_iter)
{
  icp_ptr_->setMaxCorrespondenceDistance(max_correspond_dis);
  icp_ptr_->setEuclideanFitnessEpsilon(euclid_eps);
  icp_ptr_->setTransformationEpsilon(trans_eps);
  icp_ptr_->setMaximumIterations(max_iter);

  LOG(INFO) << "ICP 的匹配参数为：" << std::endl
            << "euclid_eps: " << euclid_eps << ", "
            << "max_correspond_dis: " << max_correspond_dis << ", "
            << "trans_eps: " << trans_eps << ", "
            << "max_iter: " << max_iter << std::endl
            << std::endl;
  return true;
}

bool ICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target)
{
  icp_ptr_->setInputTarget(input_target);

  return true;
}

bool ICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, const Eigen::Matrix4f& predict_pose,
                                CloudData::CLOUD_PTR& result_cloud_ptr, Eigen::Matrix4f& result_pose)
{
  icp_ptr_->setInputSource(input_source);
  icp_ptr_->align(*result_cloud_ptr, predict_pose);
  result_pose = icp_ptr_->getFinalTransformation();
  return true;
}

float ICPRegistration::GetFitnessScore()
{
  return icp_ptr_->getFitnessScore();
}
}  // namespace lidar_localization
