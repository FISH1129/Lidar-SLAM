#include <cmath>
/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 */
#include "lidar_localization/mapping/front_end/front_end.hpp"

#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>  //直通滤波相关
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/tools/print_info.hpp"
#include <cmath>


namespace lidar_localization {
FrontEnd::FrontEnd()
    :local_map_ptr_(new CloudData::CLOUD()) {
    
    InitWithConfig();
}

bool FrontEnd::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/front_end.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------前端初始化-------------------" << std::endl;
    InitParam(config_node);
    InitRegistration(registration_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);

    return true;
}

bool FrontEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_    = config_node["local_frame_num"].as<int>();

    return true;
}

bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "前端选择的点云匹配方式为：" << registration_method << std::endl;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } 
    else if (registration_method == "NDTMANUAL"){
        registration_ptr = std::make_shared<NDTRegistrationManual>(config_node[registration_method]);
    }
    else if (registration_method == "ICP"){
        registration_ptr = std::make_shared<ICPRegistration>(config_node[registration_method]);
    }
    else if (registration_method == "ICPMANUAL"){
        registration_ptr = std::make_shared<ICPRegistrationManual>(config_node[registration_method]);
    }
    else {
        LOG(ERROR) << "没找到与 " << registration_method << " 相对应的点云匹配方式!";
        return false;
    }

    return true;
}

bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "前端" << filter_user << "选择的滤波方法为：" << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else if (filter_mothod == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
        return false;
    }

    return true;
}

bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    //当前帧点云数据与时间戳，放入帧结构体current_frame_
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices_;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices_);
    std::cout <<"原始点云"<< cloud_data.cloud_ptr->points.size() << std::endl;
    //std::cout <<"原始点云removeNaN后"<< current_frame_.cloud_data.cloud_ptr->points.size() << std::endl;

    //去除离群值添加如下：运行速度太慢了
    /*
    pcl::StatisticalOutlierRemoval<CloudData::POINT> sor;
    sor.setInputCloud(current_frame_.cloud_data.cloud_ptr);
    sor.setMeanK(50); //K近邻搜索点个数
    sor.setStddevMulThresh(1.0); //标准差倍数
	sor.setNegative(false); //保留未滤波点（内点）
	sor.filter(*current_frame_.cloud_data.cloud_ptr);  //保存滤波结果
    */

    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);
    std::cout <<"当前帧滤波后"<< filtered_cloud_ptr->points.size() << std::endl;

    //filtered_cloud_ptr是滤波后的 当前帧原始点云  加载bbox
    if (!ImportBBOXFromFile()){
      std::cout << "当前帧中没有检测到 运动及潜在运动物体！" << std::endl;
    }

    //CloudData::CLOUD_PTR object_cloud_ptr(new CloudData::CLOUD());    //一帧 存放物体点云
    //CloudData::CLOUD_PTR no_object_cloud_ptr(new CloudData::CLOUD()); //一帧 存放非物体点云

    CloudData::CLOUD_PTR temp_current_bbox_center_ptr_ (new CloudData::CLOUD());//保存当前帧每个bbox的中心点
    temp_current_bbox_center_ptr_->points.resize(40);//只要40个bbox
    std::cout << "所有bbox个数" << temp_current_bbox_center_ptr_->points.size() <<std::endl;

    for(size_t i=0; i<temp_current_bbox_center_ptr_->points.size(); i++){
        temp_current_bbox_center_ptr_->points[i].x = 0.01;
        temp_current_bbox_center_ptr_->points[i].y = 0.01;
        temp_current_bbox_center_ptr_->points[i].z = 0.01;
    }

    while(!current_bbox_.empty()){
        CloudBbox current_a_box;
        current_a_box = current_bbox_.front();
        //提取每个bbox点云，放入object_cloud_ptr
      
        if(current_a_box.score > 0.5){
            buff_mutex_.lock();

            pcl::CropBox<CloudData::POINT> region;
            std::vector<int> indices_sub;
            region.setMin(Eigen::Vector4f(-current_a_box.dx/2,	-current_a_box.dy/2,	-current_a_box.dz/2, 1));
	        region.setMax(Eigen::Vector4f( current_a_box.dx/2,	 current_a_box.dy/2,	 current_a_box.dz/2, 1));
            region.setTranslation(Eigen::Vector3f(current_a_box.x, current_a_box.y, current_a_box.z));//前三项平移
	        region.setRotation(Eigen::Vector3f(0, 0, current_a_box.heading));//旋转
            region.setInputCloud(current_frame_.cloud_data.cloud_ptr);
	        region.filter(indices_sub);//bbox里面的点在源点云下的索引
	        indices.insert(indices.end(),indices_sub.begin(),indices_sub.end());
            
            describe_current.push_back(current_a_box);

            temp_current_bbox_center_ptr_->points[bbox_id].x =  current_a_box.x;
            temp_current_bbox_center_ptr_->points[bbox_id].y =  current_a_box.y;
            temp_current_bbox_center_ptr_->points[bbox_id].z =  current_a_box.z;

            bbox_points_idx[bbox_id] = indices_sub; //bbox的id  ->  里面所有点在当前帧下的索引
            //bbox_points_idx.insert(pair<int, std::vector<int>>(bbox_id,indices_sub));
            //bbox_points_idx.insert(map<int, std::vector<int>>::value_type (bbox_id,indices_sub));
            bbox_id++;
            indices_sub.clear();

            buff_mutex_.unlock();
        }
        current_bbox_.pop_front();
    }

    bbox_id = 0;
    current_bbox_center_ptr_.reset(new CloudData::CLOUD());

    //直通滤波，去除0点
    pcl::PassThrough<CloudData::POINT> pass;
    pass.setInputCloud (temp_current_bbox_center_ptr_);
    pass.setFilterFieldName ("x"); //设置过滤时所需要的点云x字段
    pass.setFilterLimits (0.0, 0.1);//设置在过滤字段上的范围
    pass.setNegative (true);//true去除范围内的
    pass.filter (*current_bbox_center_ptr_);
    std::cout << "bbox分数高的个数" << current_bbox_center_ptr_->points.size() <<std::endl;

    /*
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices}; //创建一个内点对象，将提取到车身周围点，放到内点对象中
    for (int point : indices)
    {
      inliers->indices.push_back(point);
    }
    //点云提取
    pcl::ExtractIndices<CloudData::POINT> extract_in;
    extract_in.setInputCloud(current_frame_.cloud_data.cloud_ptr);
    extract_in.setIndices(inliers);
    extract_in.setNegative(false);  //false提取物体索引内的点
    extract_in.filter(*object_cloud_ptr);

    pcl::ExtractIndices<CloudData::POINT> extract_out;
    extract_out.setInputCloud(current_frame_.cloud_data.cloud_ptr);
    extract_out.setIndices(inliers);
    extract_out.setNegative(true);  //true提取物体索引之外的点
    extract_out.filter(*no_object_cloud_ptr);
    */

    std::vector<int>().swap(indices);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static auto last_key_frame_pose = init_pose_;

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    if (local_map_frames_.empty()) {
        current_frame_.pose = init_pose_;
        UpdateWithNewFrame(current_frame_, current_bbox_center_ptr_, describe_current); //假如current_bbox_center_ptr_为0怎么办？

        //将第一帧点云的静态权重全部赋值为1
        for (size_t i = 0; i < current_frame_.cloud_data.cloud_ptr->size(); ++i) {
            current_frame_.cloud_data.cloud_ptr->points[i].intensity = 1.0;
        }
        
        cloud_pose = current_frame_.pose;
        std::cout <<"当前帧位姿"<< std::endl;
        std::cout << cloud_pose << std::endl;

        return true;
    }

    // 不是第一帧，就正常匹配
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    //（滤波后的当前帧点云指针，预测的位姿，变换后点的指针，输出当前帧的位姿）

    std::chrono::steady_clock::time_point t1_1 = std::chrono::steady_clock::now();
    registration_ptr_->ScanMatch(filtered_cloud_ptr, 
                                 predict_pose, 
                                 result_cloud_ptr, 
                                 current_frame_.pose);
    std::chrono::steady_clock::time_point t1_2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used  = std::chrono::duration_cast<std::chrono::duration<double>>(t1_2 - t1_1);
    std::cout << "计算位姿的时间= " << time_used.count() << " seconds. " << std::endl;

    cloud_pose = current_frame_.pose; //当前帧在世界坐标系下的位姿
    std::cout <<"当前帧位姿"<< std::endl;
    std::cout << cloud_pose << std::endl;

    // 更新相邻两帧的相对运动
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (std::fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) +
        std::fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        std::fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) {

        //计算每个点的静态权重, 花费的时间
        std::chrono::steady_clock::time_point t2_1 = std::chrono::steady_clock::now();

        if(!current_bbox_center_ptr_->points.empty()) {
            CloudData::CLOUD trans_bbox;
            pcl::transformPointCloud(*current_bbox_center_ptr_, trans_bbox, cloud_pose);
            CloudData::POINT a_trans_box;
            std::vector<int> closestPointSearch; //存储目标点云中与变换后点相近的点，在目标点云中的索引
            std::vector<float> closestPointSquaredDistance; //存储近邻点对应的平方距离
            //toGetMedian getmedian;  //求距离中位数
            d.resize(current_bbox_center_ptr_->points.size());

            for (unsigned int idx = 0; idx < current_bbox_center_ptr_->points.size(); idx++) {
                a_trans_box = trans_bbox.points[idx];

                if(local_map_center_ptr.radiusSearch(a_trans_box, radius, closestPointSearch, closestPointSquaredDistance) > 0){
                    double set = 2.0;
                    for(unsigned int i = 0; i < closestPointSearch.size(); i++){
                        double up = a_trans_box.x * local_map_bbox_center_ptr_->points[closestPointSearch[i]].x +
                                    a_trans_box.y * local_map_bbox_center_ptr_->points[closestPointSearch[i]].y +
                                    a_trans_box.z * local_map_bbox_center_ptr_->points[closestPointSearch[i]].z +
                                    describe_current.at(idx).dx * describe_lm.at(closestPointSearch[i]).dx +
                                    describe_current.at(idx).dy * describe_lm.at(closestPointSearch[i]).dy +
                                    describe_current.at(idx).dz * describe_lm.at(closestPointSearch[i]).dz +
                                    describe_current.at(idx).heading * describe_lm.at(closestPointSearch[i]).heading;
                        double down = sqrt(describe_current.at(idx).x  * describe_current.at(idx).x  + 
                                           describe_current.at(idx).y  * describe_current.at(idx).y  + 
                                           describe_current.at(idx).z  * describe_current.at(idx).z  +
                                           describe_current.at(idx).dx * describe_current.at(idx).dx + 
                                           describe_current.at(idx).dy * describe_current.at(idx).dy + 
                                           describe_current.at(idx).dz * describe_current.at(idx).dz +
                                           describe_current.at(idx).heading * describe_current.at(idx).heading) * 
                                      sqrt(describe_lm.at(closestPointSearch[i]).x * describe_lm.at(closestPointSearch[i]).x +
                                           describe_lm.at(closestPointSearch[i]).y * describe_lm.at(closestPointSearch[i]).y +
                                           describe_lm.at(closestPointSearch[i]).z * describe_lm.at(closestPointSearch[i]).z +
                                           describe_lm.at(closestPointSearch[i]).dx * describe_lm.at(closestPointSearch[i]).dx +
                                           describe_lm.at(closestPointSearch[i]).dy * describe_lm.at(closestPointSearch[i]).dy +
                                           describe_lm.at(closestPointSearch[i]).dz * describe_lm.at(closestPointSearch[i]).dz +
                                           describe_lm.at(closestPointSearch[i]).heading * describe_lm.at(closestPointSearch[i]).heading);
                        std::cout << "describe_lm上面：" << describe_lm.size() << std::endl;
                        double result = up / down;
                        if((1.0-result) < set ){
                            set = 1.0 - result; 
                            d[idx] = closestPointSquaredDistance[i];
                        }
                    }
                }
                //d[idx] = closestPointSquaredDistance[0];
                //求距离的中位数
                //if(d[idx]<3.3)   //3.3->车速120km/h
                //getmedian.Insert(d[idx]);
            }
            describe_lm.clear();
            //d_median = getmedian.GetMedian();
            //cigma_D  = 1.4826 * d_median;
        }
        //得到 当前帧所有点 的静态权重
        for (size_t i = 0; i < current_frame_.cloud_data.cloud_ptr->size(); ++i) {
            current_frame_.cloud_data.cloud_ptr->points[i].intensity = 1.0;
        }

        if(!current_bbox_center_ptr_->points.empty()) {
            unsigned int idx;
            for (idx = 0; idx < current_bbox_center_ptr_->points.size(); idx++) { //current_bbox_center_ptr_->points.size()
                if (0.0 < d[idx] && d[idx]< 3.3) {
                    //current_bbox_center_ptr_->points[idx].intensity = static_cast<float>((v_0 + 1) / ((d[idx] * d[idx]) / (cigma_D * cigma_D) + v_0));
                    current_bbox_center_ptr_->points[idx].intensity = static_cast<float>(std::pow(5 / 12, d[idx]));
                    std::vector<int>::iterator it;
                    it = bbox_points_idx.at(idx).begin();
                    for (; it != bbox_points_idx.at(idx).end(); ++it) {
                        current_frame_.cloud_data.cloud_ptr->points[*it].intensity = current_bbox_center_ptr_->points[idx].intensity;
                    }
                } else {
                    current_bbox_center_ptr_->points[idx].intensity = 0.0;
                    std::vector<int>::iterator it_2 = bbox_points_idx.at(idx).begin();
                    for (; it_2 != bbox_points_idx.at(idx).end(); ++it_2) {
                        current_frame_.cloud_data.cloud_ptr->points[*it_2].intensity = current_bbox_center_ptr_->points[idx].intensity;
                    }
                }
            }
        }
        //清空vector d
        std::vector<double>().swap(d);
        //std::cout<<"清空vector d"<<std::endl;

        std::chrono::steady_clock::time_point t2_2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used  = std::chrono::duration_cast<std::chrono::duration<double>>(t2_2 - t2_1);
        std::cout << "计算静态权重时间= " << time_used.count() << " seconds. " << std::endl;
        
        UpdateWithNewFrame(current_frame_, current_bbox_center_ptr_, describe_current);
        last_key_frame_pose = current_frame_.pose;
    }

    return true;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
}

bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame, 
                                  const CloudData::CLOUD_PTR& new_center_bbox_ptr,
                                  const std::deque<CloudBbox>& new_current_bbox) {
    Frame key_frame = new_key_frame;
    CloudData::CLOUD_PTR center_bbox_ptr = new_center_bbox_ptr;
    std::deque<CloudBbox> current_bbox = new_current_bbox;
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    center_bbox_ptr.reset(new CloudData::CLOUD(*new_center_bbox_ptr));

    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR transformed_bbox_ptr(new CloudData::CLOUD());
    
    //更新局部地图(前20个关键帧组成)
    local_map_frames_.push_back(key_frame);
    local_map_bboxs_.push_back(center_bbox_ptr);
    describe_lm_.push_back(current_bbox);

    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
        local_map_frames_.pop_front();
        local_map_bboxs_.pop_front();
        describe_lm_.pop_front();
    }
    std::cout << "局部地图多少关键帧：" << local_map_frames_.size() << std::endl;

    local_map_ptr_.reset(new CloudData::CLOUD());
    local_map_bbox_center_ptr_.reset(new CloudData::CLOUD());

    //std::deque<CloudData::CLOUD_PTR>::iterator it = local_map_bboxs_.begin();
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr,
                                 *transformed_cloud_ptr,
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;//目标点云
        
        pcl::transformPointCloud(*local_map_bboxs_.at(i),
                                 *transformed_bbox_ptr,
                                 local_map_frames_.at(i).pose);
        *local_map_bbox_center_ptr_ += *transformed_bbox_ptr;//目标点云中bbox中心点
        //++it;
    }
    

    // 更新ndt匹配的目标点云
    // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
    if (local_map_frames_.size() < 10) {
        registration_ptr_->SetInputTarget(local_map_ptr_);

        if(!local_map_bbox_center_ptr_->points.empty()) {
            local_map_center_ptr.setInputCloud(local_map_bbox_center_ptr_);

            std::deque<std::deque<CloudBbox>>::iterator it = describe_lm_.begin();
            for (; it != describe_lm_.end(); ++it) {
                describe_lm.insert(describe_lm.end(), (*it).begin(), (*it).end());// describe_lm_.at(*it).end()
            }
            std::cout << "describe_lm下面：" << describe_lm.size() << std::endl;
        }
    } else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_    ->SetInputTarget(filtered_local_map_ptr);

        if(!local_map_bbox_center_ptr_->points.empty()) {
            local_map_center_ptr.setInputCloud(local_map_bbox_center_ptr_);

            std::deque<std::deque<CloudBbox>>::iterator it = describe_lm_.begin();
            for (; it != describe_lm_.end(); ++it) {
                describe_lm.insert(describe_lm.end(), (*it).begin(), (*it).end());
            }
            std::cout << "describe_lm下面：" << describe_lm.size() << std::endl;
        }
    }

    return true;
}

//加载bbox
bool FrontEnd::ImportBBOXFromFile(){
    static int current_index = 1;   //有多少帧  0018-4(原来是4); 0047-3; 0042-1; 0004-3; 0027-1

    CloudBbox init;
    init.x = 0;
    init.y = 0;
    init.z = 0;
    init.dx = 0;
    init.dy = 0;
    init.dz = 0;
    init.heading = 0;
    init.score = 0;

    std::vector<CloudBbox> current_bbox;  //当前帧点云的所有bbox

    //ros::package::getPath("lidar_localization") = /home/fbb/SLAM/myslam/MNDT_ground_cluster_4_ws/src/lidar_localization
    std::string current_bbox_file = "/media/fbb/Data/dataset/KITTI_new/bbox/0027/bbox_result/"
                                                        + std::to_string(current_index) +".txt";
    std::string current_bbox_scores_file  = "/media/fbb/Data/dataset/KITTI_new/bbox/0027/scores/"
                                                        + std::to_string(current_index) +".txt";

    std::ifstream origion_read_file;
    origion_read_file.open(current_bbox_file);

    std::ifstream bbox_scores_file;
    bbox_scores_file.open(current_bbox_scores_file);

    if (!bbox_scores_file.is_open() || !origion_read_file.is_open()){
        current_index++;
        return false;
    }

    if (bbox_scores_file.is_open())
    {
        int i = 0; //行计数器
        std::string lidae_data;
        while (getline(bbox_scores_file, lidae_data)) //逐行读取数据并存于lidae_data中，直至数据全部读取
        {
            current_bbox.push_back(init);
            std::istringstream ss(lidae_data);
            float word_data = 0.0;

            while (ss >> word_data){
                current_bbox[i].score = word_data;
            }
            i++;
        }
        /*
        int i = 0;
        float d;
        while(bbox_scores_file >> d){
            //current_bbox.push_back(init);
            current_bbox[i].score = d;
            i++;
        }
        bbox_scores_file.close();
        */
    }

    if (origion_read_file.is_open())
    {
        int i = 0; //行计数器
        std::string lidae_data;
        while (getline(origion_read_file, lidae_data, '\n')) //逐行读取数据并存于lidae_data中，直至数据全部读取
        {
          std::istringstream ss(lidae_data);
          float word_data;
          int j = 0;
          while (ss >> word_data)
          {
            if (j == 0){
              current_bbox[i].x = word_data;
            }
            else if (j == 1){
              current_bbox[i].y = word_data;
            }
            else if (j == 2){
              current_bbox[i].z = word_data;
            }
            else if (j == 3){
              current_bbox[i].dx = word_data;
            }
            else if (j == 4){
              current_bbox[i].dy = word_data;
            }
            else if (j == 5){
              current_bbox[i].dz = word_data;
            }
            else if (j == 6){
              current_bbox[i].heading = word_data;
            }
            j++;
          }
          current_bbox_.push_back(current_bbox[i]);
          i++;
        }
        std::vector<CloudBbox>().swap(current_bbox);
        current_index++;
        return true;
    }
    std::vector<CloudBbox>().swap(current_bbox);
    current_index++;
    return false;
}

}