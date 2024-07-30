

#include <pcl/filters/radius_outlier_removal.h>
#include "lidar_slam/lidar_slam.hpp"
//#include "log.hpp"
// void livox_ros::DriverNode::PointCloudDataPollThread()
// {
//   std::future_status status;
//   std::this_thread::sleep_for(std::chrono::seconds(3));
//   do {
//     lddc_ptr_->DistributePointCloudData();
//     status = future_.wait_for(std::chrono::microseconds(0));
//   } while (status == std::future_status::timeout);
// }

// void livox_ros::DriverNode::ImuDataPollThread()
// {
//   std::future_status status;
//   std::this_thread::sleep_for(std::chrono::seconds(3));
//   do {
//     lddc_ptr_->DistributeImuData();
//     status = future_.wait_for(std::chrono::microseconds(0));
//   } while (status == std::future_status::timeout);
// }

namespace lidar_slam {
LidarSlam::LidarSlam(const std::string work_path,bool localization_mode,bool offline,bool second_mapping){
    // if (!offline){
    //     start_driver(work_path);
    // }

    LidarSlam::reset(work_path,localization_mode,offline,second_mapping);

     
}


LidarSlam::LidarSlam(const LidarSlamParam yaml_param, SlamWorkMode init_mode){
    // if (!offline){
    //     start_driver(work_path);
    // }
    config_param_ = yaml_param;

    LidarSlam::reset(init_mode);

     
}



// void LidarSlam::start_driver(const std::string work_path){
//   /** Init default system parameter */
//   int xfer_format = livox_ros::kLivoxCustomMsg;
//   int multi_topic = 0;
//   int data_src = livox_ros::kSourceRawLidar;
//   double publish_freq  = 10.0; /* Hz */ //0.5~100
//   int output_type      = livox_ros::kOutputToRos;
//   std::string frame_id = "livox_frame";
//   bool lidar_bag = false;
//   bool imu_bag   = false;

//   livox_node.future_ = livox_node.exit_signal_.get_future();   

//   /** Lidar data distribute control and lidar data source set */
//  /* std::function<void(const std::shared_ptr<livox_ros::LidarMsg>&)> func_1 = [&slam](const std::shared_ptr<livox_ros::LidarMsg>& msg) {
//       slam->livox_pcl_cbk(msg);
//   };
//   std::function<void(const std::shared_ptr<livox_ros::ImuMsg>&)> func_2 = [&slam](const std::shared_ptr<livox_ros::ImuMsg>& msg) {
//       slam->imu_cbk(msg);
//   };*/
//   auto func_1 = std::bind(&LidarSlam::livox_pcl_cbk, this, std::placeholders::_1);
//   auto func_2 = std::bind(&LidarSlam::imu_cbk, this, std::placeholders::_1);
//   livox_node.lddc_ptr_ = std::make_unique<livox_ros::Lddc>(xfer_format, multi_topic, data_src, output_type,
//                         publish_freq, frame_id, lidar_bag, imu_bag,func_1,func_2);
//   livox_node.lddc_ptr_->SetRosNode(&livox_node);

//   if (data_src == livox_ros::kSourceRawLidar) {
//   //  DRIVER_INFO(livox_node, "Data Source is raw lidar.");

//     std::string user_config_path = work_path + std::string("/config/MID360_config.json");
//    // livox_node.getParam("user_config_path", user_config_path);
//   //  DRIVER_INFO(livox_node, "Config file : %s", user_config_path.c_str());

//     livox_ros::LdsLidar *read_lidar = livox_ros::LdsLidar::GetInstance(publish_freq);
//     livox_node.lddc_ptr_->RegisterLds(static_cast<livox_ros::Lds *>(read_lidar));

//     if ((read_lidar->InitLdsLidar(user_config_path))) {
//       printf("Init lds lidar successfully!\n");
//     } else {
//       printf("Init lds lidar failed!");
//     }
//   } else {
//    // DRIVER_ERROR(livox_node, "Invalid data src (%d), please check the launch file", data_src);
//   }

//   livox_node.pointclouddata_poll_thread_ = std::make_shared<std::thread>(&livox_ros::DriverNode::PointCloudDataPollThread, &livox_node);
//   livox_node.imudata_poll_thread_ = std::make_shared<std::thread>(&livox_ros::DriverNode::ImuDataPollThread, &livox_node);
// }

void LidarSlam::reset(SlamWorkMode work_mode){
    reseting = true;

    sleep(1);

    /// 激光和IMU预处理相关 *******************************************
    time_buffer.clear();               // 记录lidar时间
    lidar_buffer.clear(); //记录特征提取或间隔采样后的lidar（特征）数据
    imu_buffer.clear();
    lidar_pushed = false;
    lidar_end_time = 0;
    lidar_mean_scantime = 0.0;
    first_lidar_time = 0.0;
    scan_num = 0;
    flg_first_scan = true;
    last_timestamp_lidar = 0;
    last_timestamp_imu = -1.0;
    timediff_lidar_wrt_imu = 0.0;// TODO
    time_sync_en = false;// TODO
    timediff_set_flg = false; // 标记是否已经进行了时间补偿
    Measures = MeasureGroup();// TODO
    temp_imu_msg.clear();// TODO

    /// 点云 reset *******************************************
    UndistortCloudInOdom.reset(new PointCloudXYZI());
    undistortCloud.reset(new PointCloudXYZI());  // lidar 系
    FilteredUndistortCloud.reset(new PointCloudXYZI());
    kdtreeCloud.reset(new PointCloudXYZI());
    ObstacleCloud.reset(new PointCloudXYZI());
    FilteredObstacleCloud.reset(new PointCloudXYZI());

    /// mapping 相关 *******************************************
    unoptimized_path.clear();
    optimized_path.clear();
    T_odom_lidar = Eigen::Isometry3d::Identity();
    localization_base = Localization_base();
    current_pose = Localization_base();
    imu_file_shift = false; // TODO

    auto cloud_leaf_size = config_param_.mapping.cloud_leaf_size;
    downSizeFilterCloud.setLeafSize(cloud_leaf_size, cloud_leaf_size, cloud_leaf_size);

    auto key_frame_distance = config_param_.mapping.key_frame_distance;
    auto key_frame_angle = config_param_.mapping.key_frame_angle;
    auto loopSearchDistance = config_param_.mapping.loopSearchDistance;
    back_end.reset(new BackEnd(key_frame_distance, key_frame_angle, loopSearchDistance));

    /// sec_mapping & localizaiton ********************************
    globalLocalizationSuccess = false;

    /// important objs *******************************************
    // ikdtree
    ikdtree.reset(new KD_TREE<pcl::PointXYZINormal>());
    kf = esekfom::esekf();
    // lidar & imu 预处理
    const auto blind_distance = config_param_.lidar_preproc.blind_distance;
    const auto point_filter_num = config_param_.lidar_preproc.point_filter_num;
    const auto line_count = config_param_.lidar_preproc.line_count;
    const auto obstacle_max_range = config_param_.lidar_preproc.obstacle_max_range;
    const auto feature_enabled = config_param_.lidar_preproc.feature_enabled;
    p_lidar_pre.reset(new Preprocess());
    p_lidar_pre->set(feature_enabled, AVIA, blind_distance,point_filter_num,line_count,obstacle_max_range);
    const auto gyr_cov = config_param_.mapping.gyr_cov;
    const auto acc_cov = config_param_.mapping.acc_cov;
    const auto b_gyr_cov = config_param_.mapping.b_gyr_cov;
    const auto b_acc_cov = config_param_.mapping.b_acc_cov;
    const auto extrinT = config_param_.extrinsic.extrinT;
    const auto extrinR = config_param_.extrinsic.extrinR;
    p_imu.reset(new ImuProcess());
    p_imu->set_param(extrinT, extrinR, V3D(gyr_cov, gyr_cov, gyr_cov), V3D(acc_cov, acc_cov, acc_cov),
                                       V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov), V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    // 定位
    localization.reset(new Localization());

    
    // 线程相关 ************************************************
    if (thread!=nullptr){
        thread_run = false;
        thread->join();
        show_thread->join();
        thread_run = true;
        if(work_mode == SEC_MAPPING){
            second_mapping_thread->join();
        }
    }

    reseting = false;
    if (work_mode == MAPPING){
        thread.reset(new std::thread(&LidarSlam::loopClosureThread, this));
    }else if (work_mode == SEC_MAPPING){
        thread.reset(new std::thread(&LidarSlam::loopClosureThread, this));
        second_mapping_thread.reset(new std::thread(&LidarSlam::relocalizationForMappingThread, this));
    }else if (work_mode == LOCALIZATION){
        thread.reset(new std::thread(&LidarSlam::localizationThread, this));
    }
    show_thread.reset(new std::thread(&LidarSlam::showThread, this)); 
    working_mode_ = work_mode;
    cout << "slam reset successfully"<<endl;
}

void LidarSlam::reset(const std::string work_path,bool localization_mode,bool offline, bool second_mapping){
    cout << "this reset func has already been disabled, please use the new one"<<endl;
    return;
    {
    // reseting = true;
    
    // sleep(1);
    // time_buffer.clear();               // 记录lidar时间
    // lidar_buffer.clear(); //记录特征提取或间隔采样后的lidar（特征）数据
    // imu_buffer.clear();
    // lidar_pushed = false;
    // lidar_end_time = 0;
    // lidar_mean_scantime = 0.0;
    // first_lidar_time = 0.0;
    // scan_num = 0;
    // flg_first_scan = true;
    // last_timestamp_lidar = 0;
    // last_timestamp_imu = -1.0;
    // timediff_lidar_wrt_imu = 0.0;
    // time_sync_en = false;
    // timediff_set_flg = false; // 标记是否已经进行了时间补偿
    // unoptimized_path.clear();
    // optimized_path.clear();
    // Measures = MeasureGroup();
    // T_odom_lidar = Eigen::Isometry3d::Identity();
    // thread_run = true;
    // globalLocalizationSuccess = false;
    // imu_file_shift = false;
    // localization_base = Localization_base();
    // current_pose = Localization_base();
    // temp_imu_msg.clear();

    // ikdtree.reset(new KD_TREE<pcl::PointXYZINormal>());
    // //std::unique_ptr<std::thread> thread;

    // kf = esekfom::esekf();

        
    // std::string config_path = work_path + std::string("config/lidar_slam/mid360.yaml");
    // std::cout << "config path: " << config_path << std::endl;
    // YAML::Node config;
    // try{
    //      config = YAML::LoadFile(config_path);
    // } 
    // catch(YAML::BadFile &e) {
    //     std::cout<<"read config error!"<<std::endl;
    // }

    // sec_mapping_ = second_mapping;
    // param.localization_mode = localization_mode;
    // param.offline_mode = offline;
    // param.log_keep_time = 500;
    // std::vector<double> values =  config["mapping"]["extrinsic_T"].as<std::vector<double>>();
    // // Eigen::Map<Eigen::Vector3d>(param.extrinT.data(), values.size()) = Eigen::Map<const Eigen::VectorXd>(param.extrinT.data(), values.size());
    // param.extrinT<<values[0],values[1],values[2];
    // std::cout <<"extrinsic_T"<< param.extrinT.transpose()<<std::endl;
    // values.clear();
    // values =  config["mapping"]["extrinsic_R"].as<std::vector<double>>();
    // // Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(param.extrinR.data(), 3, 3) = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(values.data(), 3, 3);
    // param.extrinR<<values[0],values[1],values[2],values[3],values[4],values[5],values[6],values[7],values[8];
    // std::cout <<"extrinsic_R "<< param.extrinR<<std::endl;

    // values.clear();
    // values =  config["mapping"]["Lidar_In_Wheel"].as<std::vector<double>>();
    // Eigen::Matrix4d T_wheel_lidar;
    // T_wheel_lidar<<values[0],values[1],values[2],values[3],
    //         values[4],values[5],values[6],values[7],
    //         values[8],values[9],values[10],values[11],
    //         values[12],values[13],values[14],values[15];
    // param.T_wheel_lidar.matrix() = T_wheel_lidar;
    // T_lidar_wheel = param.T_wheel_lidar.inverse();
    // std::cout <<"wheel_In_lidar"<< param.T_wheel_lidar.matrix()<<std::endl;
    // std::cout <<"wheel_In_lidar rpy "<< R2ypr(param.T_wheel_lidar.matrix().block(0, 0, 3, 3)).transpose()<<std::endl;
    // // std::cout <<"wheel_In_lidar"<< T_lidar_wheel.matrix()<<std::endl;
    // // Eigen::Matrix3d rotation_matrix = param.T_wheel_lidar.matrix().block(0, 0, 3, 3);
    // // std::cout <<"ypr "<< rotation_matrix.eulerAngles(2, 1, 0)<<std::endl;

    // score_thr_ = config["global_localization"]["score_thr"].as<double>();
    // // param.load_map_path = work_path + std::string("map/") + std::string("map/") ;
    // param.load_map_path = work_path + std::string("map/");//这个参数现在未使用
    // // std::cout << "load map path: " <<  param.load_map_path << std::endl;
    // param.cloud_leaf_size = config["mapping"]["cloud_leaf_size"].as<double>();
    // param.map_leaf_size = config["ikdtree"]["map_leaf_size"].as<double>();
    // param.cube_len = config["ikdtree"]["cube_len"].as<double>();
    // param.det_range = config["ikdtree"]["det_range"].as<double>();
    // param.blind_distance = config["preprocess"]["blind"].as<double>();
    // param.point_filter_num = config["preprocess"]["point_filter_num"].as<int>();
    // param.key_frame_distance = config["mapping"]["key_frame_distance"].as<double>();
    // param.key_frame_angle = config["mapping"]["key_frame_angle"].as<double>();
    // param.loopSearchDistance = config["mapping"]["loopSearchDistance"].as<double>();
    // param.kdTreeReconstructRadius = config["ikdtree"]["kdTreeReconstructRadius"].as<double>();
    // param.kdTreeReconstructKeyFrameLeafSize = config["ikdtree"]["kdTreeReconstructKeyFrameLeafSize"].as<double>();
    // param.kdTreeReconstructPointLeafSize = config["ikdtree"]["kdTreeReconstructPointLeafSize"].as<double>();
    // param.obstacle_max_range = config["obstacle"]["max_range"].as<double>();
    // param.obstacle_min_height = config["obstacle"]["min_height"].as<double>();
    // param.obstacle_max_height = config["obstacle"]["max_height"].as<double>();
    // param.obstacle_filter_size = config["obstacle"]["filter_size"].as<double>();

    // p_lidar_pre.reset(new Preprocess());
    // p_lidar_pre->set(false,AVIA,param.blind_distance,param.point_filter_num,4,param.obstacle_max_range);
    // p_imu.reset(new ImuProcess());
    // back_end.reset(new BackEnd(param.key_frame_distance,param.key_frame_angle,param.loopSearchDistance));
    // localization.reset(new Localization());

    // UndistortCloudInOdom.reset(new PointCloudXYZI());
    // undistortCloud.reset(new PointCloudXYZI());  // lidar 系
    // FilteredUndistortCloud.reset(new PointCloudXYZI());
    // kdtreeCloud.reset(new PointCloudXYZI());
    // ObstacleCloud.reset(new PointCloudXYZI());
    // FilteredObstacleCloud.reset(new PointCloudXYZI());

    // double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;//TODO add param?
    // p_imu->set_param(param.extrinT, param.extrinR, V3D(gyr_cov, gyr_cov, gyr_cov), V3D(acc_cov, acc_cov, acc_cov),
    //                    V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov), V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    // downSizeFilterCloud.setLeafSize(param.cloud_leaf_size, param.cloud_leaf_size, param.cloud_leaf_size);
    // if (thread!=nullptr){
    //     thread_run = false;
    //     thread->join();
    //     show_thread->join();
    //     thread_run = true;
    //     if(second_mapping){
    //         second_mapping_thread->join();
    //     }
    // }

    
    // reseting = false;
    // if(!param.localization_mode){
    //     thread.reset(new std::thread(&LidarSlam::loopClosureThread, this));
    //     if (second_mapping){
    //         second_mapping_thread.reset(new std::thread(&LidarSlam::relocalizationForMappingThread, this));
    //     }
    // }
    // else{
    //     // localization->loadMap(param.load_map_path);
    //     thread.reset(new std::thread(&LidarSlam::localizationThread, this));
    // }
    // show_thread.reset(new std::thread(&LidarSlam::showThread, this));  
    // cout << "slam reset finished"<<endl;
    }
}

bool LidarSlam::sync_packages(MeasureGroup &meas) 
{
    
    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        // bool flag1 = lidar_buffer.empty();
        // bool flag2 = imu_buffer.empty();
        // cout<<"lidar_buffer.empty(): "<< flag1<<endl;
        // cout<<"imu_buffer.empty(): "<<flag2 <<endl;
        // printf("wait lidar & imu data\n");
        return false;
    }
    if (reseting == true)
       return false;
    if (lidar_pushed && omp_get_wtime()-time_buffer.front() > 1.5*0.1){
        printf("lidar lose rate %f \n",omp_get_wtime()-time_buffer.front());
    }
    /*** push a lidar scan ***/
    if (!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();         // lidar指针指向最旧的lidar数据
        meas.lidar_beg_time = time_buffer.front(); //记录最早时间

        //更新结束时刻的时间
        if (meas.lidar->points.size() <= 1) // time too little 时间太短，点数不足
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime; // 记录lidar结束时间为 起始时间 + 单帧扫描时间
            printf("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) //最后一个点的时间 小于 单帧扫描时间的一半
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime; // 记录lidar结束时间为 起始时间 + 单帧扫描时间
        }
        else
        {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000); //结束时间设置为 起始时间 + 最后一个点的时间（相对） zx 排序了么？
            // 动态更新每帧lidar数据平均扫描时间
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }
    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->time_stamp; // 最旧IMU时间
    meas.imu.clear();

    std::lock_guard<std::mutex> lk(mtx_buffer);
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) //记录imu数据，imu时间小于当前帧lidar结束时间
    {
        imu_time = imu_buffer.front()->time_stamp;
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front()); //记录当前lidar帧内的imu数据到meas.imu
        imu_buffer.pop_front();
    }
    lidar_buffer.pop_front();
    // cout<<"********************* lidar pop ************"<<endl;
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}


void LidarSlam::loopClosureThread()
{
    const int frequency = 1; // 频率为1Hz
    const std::chrono::milliseconds period(1000 / frequency);
    while (thread_run&&reseting == false)
    {
        auto start = std::chrono::steady_clock::now();
        if (loop_closure_wait)
            back_end->performLoopClosure(lidar_end_time);  //  回环检测
      //  performSCLoopClosure();
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        if (elapsed < period)
        {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}

void LidarSlam::localizationThread()
{
    const int frequency = 1.0; // 频率为1Hz
    const std::chrono::milliseconds period(1000 / frequency);
    const auto score_thr = config_param_.re_localization.score_thr;

    while (thread_run&&reseting == false)
    {
        auto start = std::chrono::steady_clock::now();
        //WorkState state;
      //  pcl::PointCloud<PointType>::Ptr temp(new pcl::PointCloud<PointType>());//TODO change to xyzi
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>());
        {
        std::lock_guard<std::mutex> lk(mtx_odom_cloud);
        pcl::copyPointCloud(*(UndistortCloudInOdom), *temp);   
        }
        if (!globalLocalizationSuccess){
            // check 
            if(!getLoadMap()){
                cout << "globalLocalization failed: map not ready ... "<<endl;
            }else if(!UndistortCloudInOdom || UndistortCloudInOdom->points.size()==0){
                cout << "globalLocalization failed: cloud empty ... "<<endl;
            // check end
            }else{
                cout <<"point(in use) count"<<UndistortCloudInOdom->points.size()<<endl;

                cout << "start globalLocalization ... "<<endl;

                //state.state("lost");
                mutex mtx_lidar_cloud;
                globalLocalizationSuccess = localization->globalLocalization(undistortCloud,T_odom_lidar,p_imu->initial_rotate,score_thr); 
                cout << "globalLocalizationSuccess: "<<globalLocalizationSuccess<<endl;

            }
            
        }
        else{
            cout << "localizing ... "<<endl;
            localization->localize(temp);

            //state.state("normal");
        }
        // state_pub_->publish(&state);

        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (elapsed < period)
        {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}

void LidarSlam::second_mapping_thread_func(){
//     const int frequency = 1.0; // 频率为1Hz
//     const std::chrono::milliseconds period(1000 / frequency);
//     const auto score_thr = config_param_.re_localization.score_thr;

//     while (thread_run&&reseting == false){

//         auto start = std::chrono::steady_clock::now();



//         auto end = std::chrono::steady_clock::now();
//         auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//         if (elapsed < period)
//         {
//             std::this_thread::sleep_for(period - elapsed);
//         }
//     }


}

void LidarSlam::relocalizationForMappingThread(){
    const int frequency = 1.0; // 频率为1Hz
    const std::chrono::milliseconds period(1000 / frequency);
    const auto score_thr = config_param_.re_localization.score_thr;

    while (thread_run&&reseting == false)
    {
        auto start = std::chrono::steady_clock::now();
        //WorkState state;
      //  pcl::PointCloud<PointType>::Ptr temp(new pcl::PointCloud<PointType>());//TODO change to xyzi
        if (!globalLocalizationSuccess){

            // check 
            if(!getLoadMap()){
                cout << "globalLocalization failed: map not ready ... "<<endl;
            }else if(!UndistortCloudInOdom || UndistortCloudInOdom->points.size()==0){
                cout << "globalLocalization failed: cloud empty ... "<<endl;
            // check end
            }else{
                cout <<"point(in use) count"<<UndistortCloudInOdom->points.size()<<endl;
                pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>());
                {
                    std::lock_guard<std::mutex> lk(mtx_odom_cloud);
                    pcl::copyPointCloud(*(UndistortCloudInOdom), *temp);   
                }

                // cout << "start globalLocalization ... "<<endl;

                //state.state("lost");
                mutex mtx_lidar_cloud;
                globalLocalizationSuccess = localization->globalLocalization(undistortCloud,T_odom_lidar,p_imu->initial_rotate, score_thr); 
                cout << "globalLocalizationSuccess: "<<globalLocalizationSuccess<<endl;

            }

        }

        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (elapsed < period)
        {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}

void LidarSlam::showThread()
{
    const int frequency = 1.0; // 频率为1Hz
    const std::chrono::milliseconds period(1000 / frequency);
    while (thread_run&&reseting == false)
    {
        auto start = std::chrono::steady_clock::now();
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        if (elapsed < period)
        {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}

void LidarSlam::livox_pcl_cbk(const std::shared_ptr<livox_ros::LidarMsg> &msg_in)
{
    if (reseting)
        return;
    
    // double preprocess_start_time = omp_get_wtime();
    // scan_count++;
    std::shared_ptr<livox_ros::LidarMsg> msg(new livox_ros::LidarMsg(*msg_in));
    if (msg->time_stamp < last_timestamp_lidar)
    {
        printf("lidar loop back, clear buffer");
        lidar_buffer.clear();
        cout<<"************************* lidar_buffer clear *********"<<endl;
    }
    /*  else if (msg->time_stamp - last_timestamp_lidar > 1.5 * 0.05){
        printf("lidar lose rate");
    }*/ 
    last_timestamp_lidar = msg->time_stamp;

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu; //????
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());

    // 特征提取或间隔采样
    p_lidar_pre->process(msg, ptr);

    {
        std::lock_guard<std::mutex> lk(mtx_obstacle_cloud);
        ObstacleCloud->points.clear();
        FilteredObstacleCloud->points.clear();
        // ObstacleCloud = transformPointCloud(p_lidar_pre->pl_obstacle, param.T_wheel_lidar);
        int size = p_lidar_pre->pl_obstacle->points.size();


    }
    std::lock_guard<std::mutex> lk(mtx_buffer);
    lidar_buffer.push_back(ptr); //储存处理后的lidar特征
    // cout<<"********************* lidar_buffer push back *********"<<endl;
    time_buffer.push_back(last_timestamp_lidar);
   // s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    
}

void LidarSlam::filter_obstacle_cloud(const PointCloudXYZI::Ptr cloud)
{
    PointCloudXYZI::Ptr wheel_cloud(new PointCloudXYZI());
    wheel_cloud = transformPointCloud(cloud, config_param_.extrinsic.T_wheel_lidar);
    const auto obstacle_max_height = config_param_.lidar_preproc.obstacle_max_height;
    const auto obstacle_min_height = config_param_.lidar_preproc.obstacle_min_height;
    for(int i = 0; i < wheel_cloud->points.size(); i++){
        if (wheel_cloud->points[i].z < obstacle_max_height && wheel_cloud->points[i].z > obstacle_min_height){
            ObstacleCloud->points.push_back(wheel_cloud->points[i]); 
        }
    }
    // 定义方格大小（5cm）
    const auto grid_size = config_param_.lidar_preproc.grid_size; // 5cm

    // 计算点云的范围
    // PointType min_point, max_point;
    // pcl::getMinMax3D(*temp_cloud, min_point, max_point);
    const auto max_x = config_param_.lidar_preproc.obstacle_max_range;
    const auto max_y = config_param_.lidar_preproc.obstacle_max_range;
    const auto min_x = -config_param_.lidar_preproc.obstacle_max_range;
    const auto min_y = -config_param_.lidar_preproc.obstacle_max_range;

    // 计算x，y方向上的方格数量
    int num_x_grids = std::ceil((max_x - min_x) / grid_size);
    int num_y_grids = std::ceil((max_y - min_y) / grid_size);

    // 创建一个二维数组来存储每个方格中的点云数量
    std::vector<std::vector<int>> grid_count(num_x_grids, std::vector<int>(num_y_grids, 0));
    // 遍历点云，将点分配到对应的方格中
    for (const auto& point : *ObstacleCloud)
    {
        int x_idx = std::floor((point.x - min_x) / grid_size);
        int y_idx = std::floor((point.y - min_y) / grid_size);

        // 检查索引是否在有效范围内
        if (x_idx >= 0 && x_idx < num_x_grids && y_idx >= 0 && y_idx < num_y_grids)
        {
            grid_count[x_idx][y_idx]++;
        }
    }

    // 创建一个新的点云来存储滤除后的点云
   

    // 遍历点云，保留符合条件的点
    const auto obstacle_filter_size = config_param_.lidar_preproc.obstacle_filter_size;
    for (const auto& point : *ObstacleCloud)
    {
        int x_idx = std::floor((point.x - min_x) / grid_size);
        int y_idx = std::floor((point.y - min_y) / grid_size);

        // 检查索引是否在有效范围内，并且方格中的点云数量大于等于10
        if (x_idx >= 0 && x_idx < num_x_grids && y_idx >= 0 && y_idx < num_y_grids && grid_count[x_idx][y_idx] >= obstacle_filter_size)
        {
            FilteredObstacleCloud->push_back(point);
        }
    }
}

void LidarSlam::livox_pcl_offline_cbk(const PointCloudXYZI::Ptr msg_in,double time_stamp)
{
    if (reseting)
        return;
    
    // double preprocess_start_time = omp_get_wtime();
    // scan_count++;
    if (time_stamp < last_timestamp_lidar)
    {
        printf("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    // else if (msg->time_stamp - last_timestamp_lidar > 1.5 * 0.05){
    //     printf("lidar lose rate");
    // }
    last_timestamp_lidar = time_stamp;

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu; //????
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }


    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr temp(new PointCloudXYZI());
    pcl::copyPointCloud(*msg_in, *ptr);
    ObstacleCloud->points.clear();
    int size = ptr->points.size();
    auto blind = config_param_.lidar_preproc.blind_distance;
    auto obstacle_max_range = config_param_.lidar_preproc.obstacle_max_range;
    if (size > 2){
        for(int i = 0; i < size; i++){
            double range = ptr->points[i].x * ptr->points[i].x + ptr->points[i].y * ptr->points[i].y + ptr->points[i].z * ptr->points[i].z;
            if (range>blind*blind && range<obstacle_max_range*obstacle_max_range)
                temp->points.push_back(ptr->points[i]);
        }
        filter_obstacle_cloud(temp);
    }
    // 创建半径滤波器对象
    // pcl::RadiusOutlierRemoval<PointType> sor;
    // sor.setInputCloud(ObstacleCloud);
    // sor.setRadiusSearch(0.1);  // 邻域半径
    // sor.setMinNeighborsInRadius(5);  // 最小邻域点数量
    // sor.filter(*FilteredObstacleCloud);

    std::lock_guard<std::mutex> lk(mtx_buffer);
    lidar_buffer.push_back(ptr); //储存处理后的lidar特征
    time_buffer.push_back(last_timestamp_lidar);
   // s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    
}

void LidarSlam::image_cbk(const cv::Mat& img,double time)
{
  //  printf("image in %f \n",time);
    for (auto it = poses_buffer.begin(); it != poses_buffer.end(); it++) {
        if(fabsf(it->first-time)<0.05){
        //  printf("use pose in %f \n",it->first);
        back_end->UpdateImage(img,it->second);
        return;
        }

    }
}

void LidarSlam::imu_cbk(const std::shared_ptr<livox_ros::ImuMsg> &msg_in)
{
    if (reseting)
        return;
    std::shared_ptr<livox_ros::ImuMsg> msg(new livox_ros::ImuMsg(*msg_in));
    if (!config_param_.common.offline_mode){
        if (!imu_file_shift){
            if (temp_imu_msg.size() > 0){
                for(auto msg : temp_imu_msg){
                    imu_file << std::fixed << std::setprecision(9) <<msg.time_stamp << " "
                        << msg.angular_velocity.x() << " "
                        << msg.angular_velocity.y() << " "
                        << msg.angular_velocity.z() << " "
                        << msg.linear_acceleration.x() << " "
                        << msg.linear_acceleration.y() << " "
                        << msg.linear_acceleration.z() << std::endl; 
                }
                temp_imu_msg.clear();
            }
            imu_file << std::fixed << std::setprecision(9) <<msg->time_stamp << " "
                << msg->angular_velocity.x() << " "
                << msg->angular_velocity.y() << " "
                << msg->angular_velocity.z() << " "
                << msg->linear_acceleration.x() << " "
                << msg->linear_acceleration.y() << " "
                << msg->linear_acceleration.z() << std::endl;
        }
        else{
            temp_imu_msg.push_back(*msg);
        }
    }
    // lidar 和 imu时间差过大，且开启 时间同步, 纠正当前输入imu的时间
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        // 对输入imu时间，纠正为 时间差 + 原始时间
        msg->time_stamp =
            (timediff_lidar_wrt_imu + msg_in->time_stamp);
    }

    double timestamp = msg->time_stamp;


    std::lock_guard<std::mutex> lk(mtx_buffer);
    if (timestamp < last_timestamp_imu)
    {
        printf("imu loop back, clear buffer\n");
        imu_buffer.clear();
    }
    else if (timestamp - last_timestamp_imu > 1.5 * 0.1){
        printf("imu lose rate\n");
    }
    imu_buffer.push_back(msg);
    // cout<<"************************* imu_buffer push back *********"<<endl;
    last_timestamp_imu = timestamp; // update imu time
    localization_wait = true;
    // if (globalLocalizationSuccess||!param.localization_mode){// TODO add lock
    if (globalLocalizationSuccess || working_mode_ == MAPPING || working_mode_ == SEC_MAPPING){// TODO add lock
        if (current_pose.base_time < localization_base.base_time - 0.005){
            // std::cout << "predicate pose "<<current_pose.imu_state.pos.transpose()<<std::endl;
            // std::cout << "update pose "<<localization_base.imu_state.pos.transpose()<<std::endl;
            current_pose = localization_base;
            for (auto it = imu_buffer.begin(); it != imu_buffer.end(); it++) {
                const std::shared_ptr<livox_ros::ImuMsg>& msg = *it;
                if (msg->time_stamp > localization_base.update_time){
                    double dt = msg->time_stamp - localization_base.update_time;
                    // if (dt < 0 || dt > 0.01)
                    //     printf("error predicate 1 %f\n",dt);
                    V3D angvel = V3D(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]) - current_pose.imu_state.bg;
                    V3D acc   = V3D(msg->linear_acceleration[0], msg->linear_acceleration[1], msg->linear_acceleration[2]) * G_m_s2 / (p_imu->mean_acc.norm());
                    acc = current_pose.imu_state.rot * (acc - current_pose.imu_state.ba) + current_pose.imu_state.grav; 
                    current_pose.imu_state.pos += current_pose.imu_state.vel * dt + 0.5 * acc * dt *dt;
                    current_pose.imu_state.vel += acc * dt;
                    current_pose.imu_state.rot = current_pose.imu_state.rot * Sophus::SO3d::exp(angvel * dt);
                    localization_base.update_time = msg->time_stamp;
                }
            }
        }
        else{
                if (msg->time_stamp > localization_base.update_time){
                    double dt = msg->time_stamp - localization_base.update_time;
                    // if (dt < 0 || dt > 0.01)
                    //     printf("error predicate 2 %f\n",dt);
                    V3D angvel = V3D(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]) - current_pose.imu_state.bg;
                    V3D acc   = V3D(msg->linear_acceleration[0], msg->linear_acceleration[1], msg->linear_acceleration[2]) * G_m_s2 / (p_imu->mean_acc.norm());
                    acc = current_pose.imu_state.rot * (acc - current_pose.imu_state.ba) + current_pose.imu_state.grav; 
                    current_pose.imu_state.pos += current_pose.imu_state.vel * dt + 0.5 * acc * dt *dt;
                    current_pose.imu_state.vel += acc * dt;
                    current_pose.imu_state.rot = current_pose.imu_state.rot * Sophus::SO3d::exp(angvel * dt);
                    localization_base.update_time = msg->time_stamp;
                }
        }
    }
    poses_buffer.push_back(std::pair(timestamp,getLidarInOdom()));
    if (poses_buffer.size() > 200)
        poses_buffer.pop_front();
    localization_wait = false;
    // if (param.localization_mode){

    // }
}


void LidarSlam::delete_log_file(double keep_time){//about 100MB pr 60s
    if (pcd_file.size() < 10)
        return;
    if (pcd_file.back() - pcd_file.front() > keep_time){
        std::filesystem::remove(config_param_.common.save_log_dir + std::to_string(pcd_file.front()) + ".pcd");
        pcd_file.pop_front();
    }
    imu_file_shift = true;
    // trim_log_file(keep_time / 60.0 * 2 * 1024 * 1024,param.save_log_path + std::string("imu_data.txt"),imu_file,pcd_file.front());
    imu_file_shift = false;
}


bool LidarSlam::run()
{
    /// 在Measure内，储存当前lidar数据及lidar扫描时间内对应的imu数据序列
    static int frame_num = 0;
    static double aver_time_consu = 0, aver_time_icp = 0,aver_time_incre = 0, aver_time_solve = 0;
    double t0, t1, t2, t3, t4, t5, match_start, solve_start,run_start,run_end;
    run_start = omp_get_wtime();
    // cout<<"lidar buffer size: "<<lidar_buffer.size()<<endl;
    // cout<<"imu   buffer size: "<<imu_buffer.size()<<endl;
    if (sync_packages(Measures))
    {
        // cout<<"sync_packages success"<<endl;
        // 第一帧lidar数据
        if (flg_first_scan)
        {
            first_lidar_time = Measures.lidar_beg_time; //记录第一帧绝对时间
            p_imu->first_lidar_time = first_lidar_time; //记录第一帧绝对时间
            flg_first_scan = false;
            cout<<"***************** flg_first_scan ********"<<endl;
            return false;
        }
        
        t0 = omp_get_wtime();
        // 根据imu数据序列和lidar数据，向前传播纠正点云的畸变, 此前已经完成间隔采样或特征提取
        {
            std::lock_guard<std::mutex> lk(mtx_lidar_cloud);
            undistortCloud->clear();
            p_imu->Process(Measures, kf, undistortCloud);
        }
        state_ikfom state_point;
        state_point = kf.get_x();
        Eigen::Isometry3d T_b_lidar(Sophus::SE3d(state_point.offset_R_L_I, state_point.offset_T_L_I).matrix());// TODO 不优化外参数就提出去
        Eigen::Isometry3d T_odom_b(Sophus::SE3d(state_point.rot, state_point.pos).matrix());
        {
            std::lock_guard<std::mutex> lk(mtx_pose);
            T_odom_lidar  =  T_odom_b * T_b_lidar;  //TODO check this
        }
        //  pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I; // global系 lidar位置
        t1 = omp_get_wtime();
        if (undistortCloud->empty() || (undistortCloud == NULL))
        {
            std::cout << "No point, skip this scan!\n"<< std::endl;
            return false;
        }

        // 检查当前lidar数据时间，与最早lidar数据时间是否足够
        bool flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < 0.1 ? false : true;

        /*** Segment the map in lidar FOV ***/
        // 动态调整局部地图,在拿到eskf前馈结果后
        ikdtree->lasermap_fov_segment(T_odom_lidar.translation()); // 根据lidar在W系下的位置，重新确定局部地图的包围盒角点，移除远端的点
        t2 = omp_get_wtime();
        /*** downsample the feature points in a scan ***/
        downSizeFilterCloud.setInputCloud(undistortCloud);//zx 0.5?
        downSizeFilterCloud.filter(*FilteredUndistortCloud);

        double feats_down_size = FilteredUndistortCloud->points.size(); //当前帧降采样后点数
        PointCloudXYZI::Ptr FilteredUndistortCloudInOdom(new PointCloudXYZI()); 
        double filter_time = omp_get_wtime();
        /*** initialize the map kdtree ***/
        if (ikdtree->Root_Node == nullptr)
        {
            if (feats_down_size > 5)
            {
                ikdtree->set_downsample_param(config_param_.ikdtree.map_leaf_size);//0.5 默认0.2
                ikdtree->set_cube_len(config_param_.ikdtree.cube_len);
                ikdtree->set_det_range(config_param_.ikdtree.det_range);

                FilteredUndistortCloudInOdom->resize(feats_down_size);
                //   for (int i = 0; i < feats_down_size; i++)
                //   {
                FilteredUndistortCloudInOdom = transformPointCloud(FilteredUndistortCloud, T_odom_lidar); // point转到odom系下
                //   }
                // world系下对当前帧降采样后的点云，初始化lkd-tree
                ikdtree->Build(FilteredUndistortCloudInOdom->points);
            }
            std::cout << "build ikdtree! "<<feats_down_size<< std::endl;
            return false;
        }

        int featsFromMapNum = ikdtree->validnum();
        int kdtree_size_st = ikdtree->size();
        
        // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

        /*** ICP and iterated Kalman filter update ***/
        if (feats_down_size < 5)
        {
            std::cout <<"No point after filter, skip this scan!"<<std::endl;
            return false;
        }
        FilteredUndistortCloudInOdom->resize(feats_down_size);

        /* if (true) // If you need to see map point, change to "if(1)" //zx delete this publish
        {
            PointVector().swap(ikdtree.PCL_Storage);
            ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
            kdtreeCloud->clear();
            kdtreeCloud->points = ikdtree.PCL_Storage;
            // publish_map(pubLaserCloudMap);
        }*/
        vector<PointVector> Nearest_Points;
        Nearest_Points.resize(feats_down_size);

        /*** iterated state estimation ***/
        double t_update_start = omp_get_wtime();
        kf.update_iterated_dyn_share_modified(0.001, FilteredUndistortCloud, *ikdtree, Nearest_Points, 4, false);
        double t_update_end = omp_get_wtime();
        state_point = kf.get_x();
        T_b_lidar = Sophus::SE3d(state_point.offset_R_L_I, state_point.offset_T_L_I).matrix();
        T_odom_b = Sophus::SE3d(state_point.rot, state_point.pos).matrix();  
        T_odom_lidar  =  T_odom_b * T_b_lidar;  //TODO check this
        //  double t_update_end = omp_get_wtime();
        //   while(localization_wait){ // TODO check

        //   }
        localization_base.imu_state = state_point;
        localization_base.base_time = lidar_end_time;
        localization_base.update_time = lidar_end_time;
        // if (!param.localization_mode){
        if (working_mode_==MAPPING || working_mode_ == SEC_MAPPING){
            loop_closure_wait = true;
            bool insert = back_end->saveKeyFramesAndFactor(T_odom_lidar,undistortCloud,lidar_end_time); // TODO add transform
            if (insert){
                cout<<"************* keyPosesCount: "<<back_end->getKeyframePoses().size()<<endl;
                back_end->saveCurrentCloud(undistortCloud,getLidarInMap());//注意这里只是为了取水平面，后端还是在odom坐标系
                {
                    std::lock_guard<std::mutex> lk(mtx_path);
                    unoptimized_path.emplace_back(getWheelInMap());//TODO max size
                    if (unoptimized_path.size() > 200)
                        unoptimized_path.pop_front();
                }
                T_odom_lidar = back_end->getCurrentPose().pose;
                T_odom_b = T_odom_lidar * T_b_lidar.inverse();
                state_ikfom state_updated = kf.get_x();
                state_updated.pos = T_odom_b.translation();
                state_updated.rot =  Sophus::SO3d(T_odom_b.rotation());
                kf.change_x(state_updated); 
            }

            // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹， 重构ikdtree
            bool LoopIsClosed = back_end->correctPoses();
            {
                std::lock_guard<std::mutex> lk(mtx_path);
                optimized_path.clear();
                std::vector<KeyPose> lidar_in_odom;
                lidar_in_odom = back_end->getKeyframePoses();
                for(int i = 0;i < lidar_in_odom.size();i++){//TODO max size
                    optimized_path.emplace_back(getOdomToMap() * lidar_in_odom[i].pose * T_lidar_wheel);
                }
            }
            if(LoopIsClosed){
                back_end->recontructIKdTree(*ikdtree,
                                                config_param_.ikdtree.kdTreeReconstructRadius,
                                                config_param_.ikdtree.kdTreeReconstructKeyFrameLeafSize,
                                                config_param_.ikdtree.kdTreeReconstructPointLeafSize);
            }
            loop_closure_wait = false;
        }else{
                {
                    std::lock_guard<std::mutex> lk(mtx_path);
                    unoptimized_path.emplace_back(getWheelInMap());//TODO max size
                    if (unoptimized_path.size() > 200)
                        unoptimized_path.pop_front();
                }             
        }
        // std::cout<<"test "<< R2ypr(T_odom_lidar.matrix().block<3, 3>(0, 0)).transpose() << std::endl;
        // if (!param.localization_mode){
        if (working_mode_==MAPPING || working_mode_ == SEC_MAPPING){

        }


        {
            std::lock_guard<std::mutex> lk(mtx_odom_cloud);
            UndistortCloudInOdom->resize(undistortCloud->points.size());
            UndistortCloudInOdom = transformPointCloud(undistortCloud, T_odom_lidar); 
        }
        t3 = omp_get_wtime();

        /*** add the feature points to map kdtree ***/
        FilteredUndistortCloudInOdom = transformPointCloud(FilteredUndistortCloud, T_odom_lidar);
        
        t4 = omp_get_wtime();
        ikdtree->map_incremental(FilteredUndistortCloudInOdom,Nearest_Points,flg_EKF_inited);
        t5 = omp_get_wtime();
        {
            frame_num++;
            int kdtree_size_end = ikdtree->size();
            // std::cout <<"??? " <<aver_time_consu * (frame_num - 1) / frame_num << " "<<(t5 - t0) / frame_num<< " "<<frame_num<< std::endl;
            aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
            aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;

            //   printf("[ mapping ]: time: IMU process: %0.6f,kdtree size %d test %0.6f,test1 %0.6f, ave ICP: %0.6f, map incre: %0.6f ave total: %0.6f \n"
            //   , t1 - t0, kdtree_size_end, filter_time - t2,t3 - t_update_end, aver_time_icp, t5 - t4, aver_time_consu);
        }
        run_end =  omp_get_wtime();
        if (run_end - run_start > 0.1)
            printf("lidar slam lose rate");
        
        return true;
    }
    else{
        // cout << "sync measure failed !"<<endl;
        delete_log_file(config_param_.common.log_keep_time);
    }

    return false;

    
}
}

