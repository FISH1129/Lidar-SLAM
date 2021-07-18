#ifndef CPU_NDT_H_
#define CPU_NDT_H_

#include <memory>
#include <vector>
#include "Registration.h"
#include "VoxelGrid.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include "lidar_localization/sensor_data/cloud_data.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <std_msgs/Header.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <sensor_msgs/PointCloud2.h>


//#include "lidar_localization/mapping/front_end/front_end_flow.hpp"

namespace lidar_localization {

//template <typename PointSourceType, typename PointTargetType>
class NormalDistributionsTransform: public Registration{
public:
	NormalDistributionsTransform();

	NormalDistributionsTransform(const NormalDistributionsTransform &other);	

	void setStepSize(double step_size);
	void setResolution(float resolution);// 设置 ndt 中 voxel 的大小
	void setOutlierRatio(double olr);// 设置离群点的比例，用于计算混合分布中均值和高斯的权重

	double getStepSize() const;
	float getResolution() const;
	double getOutlierRatio() const;

	double getTransformationProbability() const;

	int getRealIterations();

	/* Set the input map points */
	//void setInputSource(const const CloudData::CLOUD_PTR &input);
	//void setInputSource(CloudData::CLOUD_PTR input) override;
	//void oriSetInputTarget(CloudData::CLOUD_PTR input);//最后计算得分时用原来的网格划分
	void setInputTarget(CloudData::CLOUD_PTR input) override;

	/* Compute and get fitness score */
	double getFitnessScore(double max_range = DBL_MAX);
	// 调用 voxel_grid_ 的 update 进行更新
	void updateVoxelGrid(CloudData::CLOUD_PTR new_cloud);

	//using Ptr = std::shared_ptr<NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>>;
	//using Ptr = std::shared_ptr<NormalDistributionsTransform<PointSourceType, PointTargetType>>;
	//using Ptr = std::shared_ptr<NormalDistributionsTransform<CloudData::POINT,CloudData::POINT>>;

protected:
	// 给定初始位姿估计，牛顿迭代计算位姿
	void computeTransformation(const Eigen::Matrix<float, 4, 4> &guess) override;

	using Registration::transformation_epsilon_;
	using Registration::max_iterations_;
	using Registration::source_cloud_;
	using Registration::trans_cloud_;
	using Registration::converged_;
	using Registration::nr_iterations_;
	using Registration::final_transformation_;
	using Registration::transformation_;
	using Registration::previous_transformation_;
	using Registration::target_cloud_updated_;
	using Registration::target_cloud_;

private:
	//Copied from ndt.h
    double auxilaryFunction_PsiMT (double a, double f_a, double f_0, double g_0, double mu = 1.e-4);

    //Copied from ndt.h
    double auxilaryFunction_dPsiMT (double g_a, double g_0, double mu = 1.e-4);

    double updateIntervalMT (double &a_l, double &f_l, double &g_l,
							 double &a_u, double &f_u, double &g_u,
							 double a_t, double f_t, double g_t);

    double trialValueSelectionMT (double a_l, double f_l, double g_l,
								  double a_u, double f_u, double g_u,
								  double a_t, double f_t, double g_t);

	void computeAngleDerivatives(Eigen::Matrix<double, 6, 1> pose, 
								 bool compute_hessian = true);

	double computeStepLengthMT(const Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> &step_dir,
								double step_init, double step_max, double step_min, double &score,
								Eigen::Matrix<double, 6, 1> &score_gradient, Eigen::Matrix<double, 6, 6> &hessian,
								CloudData::CLOUD &trans_cloud);

	void computeHessian(Eigen::Matrix<double, 6, 6> &hessian, 
						CloudData::CLOUD &trans_cloud, 
						Eigen::Matrix<double, 6, 1> &p);

	double computeDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient, 
							  Eigen::Matrix<double, 6, 6> &hessian,
							  CloudData::CLOUD &trans_cloud,
							  Eigen::Matrix<double, 6, 1> pose, 
							  bool compute_hessian = true);
	void computePointDerivatives(Eigen::Vector3d &x, 
								 Eigen::Matrix<double, 3, 6> &point_gradient, 
								 Eigen::Matrix<double, 18, 6> &point_hessian, 
								 bool computeHessian = true);
	double updateDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient, 
							 Eigen::Matrix<double, 6, 6> &hessian,
							 Eigen::Matrix<double, 3, 6> point_gradient, 
							 Eigen::Matrix<double, 18, 6> point_hessian,
							 Eigen::Vector3d &x_trans, 
							 Eigen::Matrix3d &c_inv, 
							 bool compute_hessian = true);
	void updateHessian(Eigen::Matrix<double, 6, 6> &hessian,
					   Eigen::Matrix<double, 3, 6> point_gradient, 
					   Eigen::Matrix<double, 18, 6> point_hessian,
					   Eigen::Vector3d &x_trans, 
					   Eigen::Matrix3d &c_inv);

	double gauss_d1_, gauss_d2_;
	double outlier_ratio_;
	Eigen::Vector3d j_ang_a_, j_ang_b_, j_ang_c_, j_ang_d_, j_ang_e_, j_ang_f_, j_ang_g_, j_ang_h_;

	Eigen::Vector3d h_ang_a2_, h_ang_a3_,
					h_ang_b2_, h_ang_b3_,
					h_ang_c2_, h_ang_c3_,
					h_ang_d1_, h_ang_d2_, h_ang_d3_,
					h_ang_e1_, h_ang_e2_, h_ang_e3_,
					h_ang_f1_, h_ang_f2_, h_ang_f3_;
	
	// [x,y,z,roll,pitch,yaw] 的最小变化量(m, rad)，当小于这个值时就停止 align
	// double transformation_epsilon;
	// More-Thuente line search 的最大步长，大一些可以更快的下降，但也可能 overshoot 导致陷入局部最优
	double step_size_;
	
	// ndt 中 voxel 的大小，每个 voxel 中会保存 mean，covariance 和 点云，这个值是最 scale dependent 的，
	//应该足够大（一个 voxel 至少包含 6 个点），也不能太大（要反映局部空间的特征）
	float resolution_;
	double trans_probability_;

	int real_iterations_;


	VoxelGrid voxel_grid_;

	//添加的
	struct Detected_Obj{
        jsk_recognition_msgs::BoundingBox bounding_box_;

        CloudData::POINT min_point_;
        CloudData::POINT max_point_;
        CloudData::POINT centroid_;
		double w_cs; //该类的静态权重 = 所有点的权重的均值
        Eigen::Matrix3d covMat = Eigen::Matrix3d::Identity();
    };

    std_msgs::Header point_cloud_header_;
    
	std::vector<Detected_Obj> global_obj_list;
	std::vector<double> seg_distance_, cluster_distance_;
	//pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_closest_ptr_;
	//pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_ptr_;
	//pcl::KdTreeFLANN<CloudData::POINT> kdtree; //找最近聚类

	CloudData::CLOUD_PTR cluster_mean_;
	CloudData::CLOUD_PTR input_target_;


    int MIN_CLUSTER_SIZE = 50;  
    int MAX_CLUSTER_SIZE = 25000;
	int K = 1;
	int index = 0;

	//计算点的静态权重
	float radius = 10.0;
	int u_D = 0;    //均值为0,表示距离越小，权重越大

public:
	//添加的
	ros::Publisher pub_bounding_boxs_;
	void cluster_by_distance(CloudData::CLOUD_PTR in_pc, std::vector<Detected_Obj> &obj_list);
	void cluster_segment(CloudData::CLOUD_PTR in_pc,double in_max_cluster_distance, std::vector<Detected_Obj> &obj_list);
	void computeStaticWeight(CloudData::CLOUD &trans_cloud);

};
}

#endif
