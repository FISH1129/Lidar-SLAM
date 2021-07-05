/*
 * @Description: ICPManual 匹配模块
 * @Author: Zm Liu
 * @Date:
 */
#include "lidar_localization/models/registration/ndt_registration_manual.hpp"
#include <Eigen/Core>
#include "glog/logging.h"

namespace lidar_localization
{
NDTRegistrationManual::NDTRegistrationManual(const YAML::Node& node)
{
    auto res           = node["res"].as<float>();
    auto step_size     = node["step_size"].as<float>();
    auto trans_eps     = node["trans_eps"].as<float>();
    auto max_iter      = node["max_iter"].as<int>();
    auto outlier_ratio = node["outlier_ratio"].as<float>();

  SetRegistrationParam(res, step_size, trans_eps, max_iter, outlier_ratio);
}

NDTRegistrationManual::NDTRegistrationManual(float res, float step_size, float trans_eps, int max_iter, float outlier_ratio)
  //: ndt_manual_object(new lidar_localization::NormalDistributionsTransform<CloudData::POINT,CloudData::POINT>)
{
  SetRegistrationParam(res, step_size, trans_eps, max_iter, outlier_ratio);
}

bool NDTRegistrationManual::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter, float outlier_ratio)
{
  ndt_manual_object.setResolution(res);      //分辨率 VoxelGrid 的 leaf_size
  ndt_manual_object.setStepSize(step_size);  //More-Thuente线搜索的最大更新步长
  ndt_manual_object.setTransformationEpsilon(trans_eps); //规定最小变化量（最小的Δ[x,y,z,roll,pitch,yaw]）
  ndt_manual_object.setMaximumIterations(max_iter); // 最大迭代次数
  ndt_manual_object.setOutlierRatio(outlier_ratio); //离群值比例

  std::cout << "NDT Manual 的匹配参数为：" << std::endl
            << "res: " << res << ", "
            << "step_size: " << step_size << ", "
            << "trans_eps: " << trans_eps << ", "
            << "max_iter: " << max_iter << ", "
            << "outlier_ratio: " << outlier_ratio 
            << std::endl
            << std::endl;
  return true;
}

bool NDTRegistrationManual::SetInputTarget(const CloudData::CLOUD_PTR &input_target)
{
    ndt_manual_object.setInputTarget(input_target);
    //ndt_manual_object.oriSetInputTarget(input_target);

    return true;
}


bool NDTRegistrationManual::ScanMatch(const CloudData::CLOUD_PTR &input_source,
                                      const Eigen::Matrix4f &predict_pose,
                                      CloudData::CLOUD_PTR &result_cloud_ptr,
                                      Eigen::Matrix4f &result_pose)
{
    ndt_manual_object.setInputSource(input_source);
    ndt_manual_object.align(*result_cloud_ptr, predict_pose);
    result_pose = ndt_manual_object.getFinalTransformation();

    //ndt_manual_object.computeStaticWeight(CloudData::CLOUD &trans_cloud);
    return true;
}

float NDTRegistrationManual::GetFitnessScore()
{
  //因为原本的得分函数计算，需要用到 voxel_grid_.nearestNeighborDistance (这个在目标点云聚类后，没有了)
  //std::cout << "暂时未实现 加入静态权重匹配 的得分函数计算" << std::endl;
  
  return static_cast<float>(ndt_manual_object.getFitnessScore());
}

}  // namespace lidar_localization
