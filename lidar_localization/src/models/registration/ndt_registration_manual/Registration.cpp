#include "lidar_localization/models/registration/ndt_registration_manual/Registration.h"
#include "lidar_localization/models/registration/ndt_registration_manual/debug.h"
#include <iostream>

namespace lidar_localization {

////template <typename CloudData::POINT, typename CloudData::POINT>
Registration::Registration()
{
	max_iterations_ = 0;

	converged_ = false;
	nr_iterations_ = 0;

	transformation_epsilon_ = 0;//规定最小变化量的（也就是最小的Δ[x,y,z,roll,pitch,yaw]）
	target_cloud_updated_ = true;

	trans_cloud_.points.clear();
}

//template <typename CloudData::POINT, typename CloudData::POINT>
Registration::~Registration(){
	return;
}

//template <typename CloudData::POINT, typename CloudData::POINT>
void Registration::setTransformationEpsilon(double trans_eps){
	transformation_epsilon_ = trans_eps;
}

//template <typename CloudData::POINT, typename CloudData::POINT>
double Registration::getTransformationEpsilon() const{
	return transformation_epsilon_;
}

//template <typename CloudData::POINT, typename CloudData::POINT>
void Registration::setMaximumIterations(int max_itr){
	max_iterations_ = max_itr;
}

//template <typename CloudData::POINT, typename CloudData::POINT>
int Registration::getMaximumIterations() const{
	return max_iterations_;
}

//template <typename CloudData::POINT, typename CloudData::POINT>
Eigen::Matrix<float, 4, 4> Registration::getFinalTransformation() const
{
	return final_transformation_;
}

//template <typename CloudData::POINT, typename CloudData::POINT>
int Registration::getFinalNumIteration() const
{
	return nr_iterations_;
}

//template <typename CloudData::POINT, typename CloudData::POINT>
bool Registration::hasConverged() const
{
	return converged_;
}

//template <typename CloudData::POINT, typename CloudData::POINT>
void Registration::setInputSource(const CloudData::CLOUD_PTR input){
	source_cloud_ = input;
}

//设置目标点云
//template <typename CloudData::POINT, typename CloudData::POINT>
void Registration::setInputTarget(const CloudData::CLOUD_PTR input){
	target_cloud_ = input;
}

//template <typename CloudData::POINT, typename CloudData::POINT>
void Registration::align(const Eigen::Matrix<float, 4, 4> &guess)
{
	converged_ = false;

	final_transformation_ = transformation_ = previous_transformation_ = Eigen::Matrix<float, 4, 4>::Identity();

	trans_cloud_.points.resize(source_cloud_->points.size());

	for (unsigned int i = 0; i < trans_cloud_.points.size(); i++) {
		trans_cloud_.points[i] = source_cloud_->points[i];
	}

	computeTransformation(guess);
}

//template <typename CloudData::POINT, typename CloudData::POINT>
void Registration::align(CloudData::CLOUD &output, const Eigen::Matrix<float, 4, 4> &guess)
{
	align(guess);
}

//不执行这个computeTransformation，已经被派生类覆盖
//template <typename CloudData::POINT, typename CloudData::POINT>
void Registration::computeTransformation(const Eigen::Matrix<float, 4, 4> &guess) {
	printf("Unsupported by Registration\n");
}

}
