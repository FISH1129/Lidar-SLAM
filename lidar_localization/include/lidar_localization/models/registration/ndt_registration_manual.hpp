/*
 * @Description: ICP 匹配模块
 * @Author: Zm Liu
 * @Date:
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_MANUAL_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_MANUAL_HPP_

#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/registration/ndt_registration_manual/NormalDistributionsTransform.h"
#include "sophus/se3.hpp"

namespace lidar_localization
{
    class NDTRegistrationManual : public RegistrationInterface
    {
    public:
        explicit NDTRegistrationManual(const YAML::Node &node);
        NDTRegistrationManual(float res, float step_size, float trans_eps, int max_iter, float outlier_ratio);

        bool SetInputTarget(const CloudData::CLOUD_PTR &input_target) override;
        bool ScanMatch(const CloudData::CLOUD_PTR &input_source,
                       const Eigen::Matrix4f &predict_pose,
                       CloudData::CLOUD_PTR &result_cloud_ptr,
                       Eigen::Matrix4f &result_pose) override;
        float GetFitnessScore() override;


    private:
        bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter, float outlier_ratio);

    private:
        //lidar_localization::NormalDistributionsTransform<CloudData::POINT,CloudData::POINT>::Ptr ndt_manual_ptr_;
        lidar_localization::NormalDistributionsTransform ndt_manual_object;
    };
} // namespace lidar_localization

#endif