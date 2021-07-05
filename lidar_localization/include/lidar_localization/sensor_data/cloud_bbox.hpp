/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:25:13
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_BBOX_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_BBOX_HPP_


namespace lidar_localization
{
  class CloudBbox
  {
    public:
      float x  = 0.0;
      float y  = 0.0;
      float z  = 0.0;
      float dx = 0.0;
      float dy = 0.0;
      float dz = 0.0;
      float heading = 0.0;

      float score =0.0;
  };
}

#endif