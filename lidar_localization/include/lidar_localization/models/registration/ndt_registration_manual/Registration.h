#ifndef CPU_REG_H_
#define CPU_REG_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {

//template <typename CloudData::POINT, typename CloudData::POINT>
class Registration {
public:
	Registration();

	void align(const Eigen::Matrix<float, 4, 4> &guess);

	void align(CloudData::CLOUD &output, const Eigen::Matrix<float, 4, 4> &guess);

	void setTransformationEpsilon(double trans_eps);

	double getTransformationEpsilon() const;

	void setMaximumIterations(int max_itr);

	int getMaximumIterations() const;

	Eigen::Matrix<float, 4, 4> getFinalTransformation() const;

	virtual /* Set input Scanned point cloud.
	 * Simply set the point cloud input_ */
	void setInputSource(CloudData::CLOUD_PTR input);

	virtual /* Set input reference map point cloud.
	 * Set the target point cloud ptr target_cloud_ */
	void setInputTarget(CloudData::CLOUD_PTR input);

	int getFinalNumIteration() const;

	bool hasConverged() const;

	void updateInputTarget(const CloudData::CLOUD_PTR new_cloud);

	virtual ~Registration();

protected:
	//virtual声明虚函数，在派生类中对该函数覆盖，以实现运行时多态性
	virtual void computeTransformation(const Eigen::Matrix<float, 4, 4> &guess);

	double transformation_epsilon_;
	int max_iterations_;

	//Original scanned point clouds
	CloudData::CLOUD_PTR source_cloud_;

	//Transformed point clouds
	CloudData::CLOUD trans_cloud_;

	bool converged_;
	int nr_iterations_;
	
	//结果保存在 final_transformation_ 
	Eigen::Matrix<float, 4, 4> final_transformation_, transformation_, previous_transformation_;

	bool target_cloud_updated_;

	// Reference map point
	CloudData::CLOUD_PTR target_cloud_;
};
}

#endif
