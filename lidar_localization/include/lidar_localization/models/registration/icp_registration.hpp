/*
 * @Description: ICP 匹配模块
 * @Author: Zm Liu
 * @Date:
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_

#include <pcl/registration/icp.h>
#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization
{
class ICPRegistration : public RegistrationInterface
{
public:
    ICPRegistration(const YAML::Node &node);
    ICPRegistration(float euclid_eps, float max_correspond_dis, float trans_eps, int max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR &input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR &input_source,
                   const Eigen::Matrix4f &predict_pose,
                   CloudData::CLOUD_PTR &result_cloud_ptr,
                   Eigen::Matrix4f &result_pose) override;
    float GetFitnessScore() override;

private:
    bool SetRegistrationParam(float euclid_eps, float max_correspond_dis, float trans_eps, int max_iter);

private:
    pcl::IterativeClosestPoint<CloudData::POINT,CloudData::POINT>::Ptr icp_ptr_;
};
} // namespace lidar_localization

#endif