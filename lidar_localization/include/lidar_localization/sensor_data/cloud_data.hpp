/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:17:49
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct VelodynePointXYZILW
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    //uint16_t ring;    // 使用了 ring 信息，雷达数据位于第几条扫描线上
    //uint16_t label;   // 点是否属于检测物体   1:是 0：不是
    //float time;       //相对于该帧初始点的时间
    float s_weight;   //该点的静态权重， 1:静态, 0:动态
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Eigen的字段对齐
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZILW,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    //(uint16_t, ring,  ring)
    //(unit16_t, label, label)
    //(float, time, time)
    (float, s_weight, s_weight)
);

namespace lidar_localization {

class CloudData {
  public:
    using POINT = pcl::PointXYZI;
    //using POINT = VelodynePointXYZILW;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

  public:
    CloudData() //构造函数
      :cloud_ptr(new CLOUD()) {
    }

  public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};
}

#endif