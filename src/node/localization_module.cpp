#include <ros/ros.h>
#include "node/localization_module.h"

namespace localization_module {
LocalizationModule::LocalizationModule(const std::string work_path, ModuleStatus init_status){
    curr_dir_ = work_path;
    ROS_INFO("Current directory: %s", curr_dir_.c_str());

    // // load_params();
    if (!load_lidar_slam_param()){
        ROS_ERROR("Load lidar-slam param failed!");
    }else {
        ROS_INFO("Load lidar-slam param successfully!");
    }

    if(!create_ROS_IO()){
        ROS_ERROR("Create ROS-IO failed!");
    }else {
        ROS_INFO("Create ROS-IO successfully!");
    }

    //************************** TODO: 待确认 *******************************
    if (show_rviz_){// this param load from lasunch file
        show_thread_ = std::thread(&LocalizationModule::show_thread, this);
        ROS_INFO("Show_thread started");
    }

    // TODO
    std::thread load_data_thread;
    if (offline_mode_){
        // 读取文件夹中的文件名
    }
    //***********************************************************************

    if(!init_module_by_set_status(init_status)){
        ROS_INFO("Try to init module with status: %s, but failed",print_ModuleStatus(init_status).c_str());
    }else{
        ROS_INFO("Localization Module Start with status: %s", print_ModuleStatus(running_module_status_).c_str());
    }


    ROS_INFO("***************************************************");
}

LocalizationModule::~LocalizationModule(){

}

void LocalizationModule::localization_module_ctrl_cbk(const std_msgs::UInt32 &msg_in){
    /** msg_in
     * enum SlamCtrlCmd{
     *     START_MAPPING           = 1000,  // 开始建图
     *     START_SEC_MAPPING       = 2000,  // 重定位->建图，二次建图
     *     MAPPING_POINT_BEGIN     = 3000,  // 设置起点
     *     MAPPING_ELE_DELETE      = 4000,  // 创建地图元素过程中，清除当前元素（当前元素还未完成创建）
     *     MAPPING_POINT_END       = 5000,  // 设置终点，带子地图ID，5001，ID=1
     *     EXIT_MAPPING            = 6000,  // 退出建图
     *     START_LOCALIZATION      = 7000,  // 重定位->定位
     *     EXIT_LOCALIZATION       = 8000,  // 退出定位
     *     CMD_MAX
     * };
     *      
     * to be continued
     */  
    //// topic-name: "/mapping_manager_cmd"

    auto msg = msg_in;
    int ctrl_type = msg.data/1000 * 1000;
    auto curr_cmd = static_cast<SlamCtrlCmd>(ctrl_type);
    ROS_INFO("Received Ctrl Cmd: %s", print_SlamCtrlCmd(curr_cmd).c_str());

    switch (curr_cmd){
        case START_MAPPING:{// 初次建图，或重置后建图
            last_running_module_status_ = running_module_status_;
            set_module_status_ = MODULE_MAPPING;
            running_module_status_ = MODULE_STARTING_SLAM;
            if(start_mapping(set_module_status_)){//启动建图成功
                running_module_status_ = set_module_status_;
                mapping_status_ = M_STANDBY;
            }else{//启动建图失败
                running_module_status_ = last_running_module_status_;
                set_module_status_ = last_running_module_status_;// 设置失败，则恢复 set_module_status_ 状态
            }
            last_running_module_status_ = MODULE_IDLE;// 复位为空

            break;
        }
        case START_SEC_MAPPING:{// 重定位，并开始建图，/// TODO/////////////////////////////////////
            int map_id = msg.data % 1000;
            last_running_module_status_ = running_module_status_;
            set_module_status_ = MODULE_SEC_MAPPING;
            running_module_status_ = MODULE_STARTING_SLAM;
            if(start_second_mapping(set_module_status_, map_id)){
                running_module_status_ = set_module_status_;
                mapping_status_ = M_STANDBY;
            }else{
                running_module_status_ = last_running_module_status_;
                // set_module_status_ = last_running_module_status_;
            }
            set_module_status_ = MODULE_IDLE;// 复位为空
            last_running_module_status_ = MODULE_IDLE;// 复位为空
            break;
        }
        case MAPPING_POINT_BEGIN:{// 设置起点
            if(mark_start_point())
                mapping_status_ = M_CREATING_ELE;
            break;
        }
        case MAPPING_ELE_DELETE:{// 删除当前元素
            if(clear_curr_element())
                mapping_status_ = M_STANDBY;
            break;
        }
        case MAPPING_POINT_END:{// 设置终点
            int ele_id = msg.data % 1000;
            if(mark_end_point(ele_id))
                mapping_status_ = M_STANDBY;
            break;
        }
        case EXIT_MAPPING:{// 退出建图
            last_running_module_status_ = running_module_status_;
            set_module_status_ = MODULE_IDLE;
            running_module_status_ = MODULE_STOPPING_SLAM;
            if(stop_mapping()){
                running_module_status_ = MODULE_IDLE;
                mapping_status_ = M_INACTIVE;
            }else{
                running_module_status_ = last_running_module_status_;
                set_module_status_ = running_module_status_;
            }
            last_running_module_status_ = MODULE_IDLE;
            break;
        }
        case START_LOCALIZATION:{// 开始定位，（先重定位，再定位）
            int map_id = msg.data % 1000;
            last_running_module_status_ = running_module_status_;
            set_module_status_ = MODULE_LOCALIZATION;
            running_module_status_ = MODULE_STARTING_SLAM;
            if(start_localization(set_module_status_, map_id)){
                running_module_status_ = MODULE_LOCALIZATION;
                // localization_status_ = L_RELOCALIZING;
            }else{
                set_module_status_ = running_module_status_;
            }
            last_running_module_status_ = MODULE_IDLE;
            break;
        }
        case EXIT_LOCALIZATION:{// 退出定位
            last_running_module_status_ = running_module_status_;
            set_module_status_ = MODULE_IDLE;
            running_module_status_ = MODULE_STOPPING_SLAM;
            if(stop_localization()){// 停止定位成功
                running_module_status_ = MODULE_IDLE;
            }
            else{
                set_module_status_ = running_module_status_;
            }

            last_running_module_status_ = MODULE_IDLE;
            break;
        }
        default:{
            ROS_INFO("mapping ctrl msg: %u invalid!", msg.data);
            break;
        }
    }
}

bool LocalizationModule::start_mapping(ModuleStatus set_status){
    ROS_INFO("Module Status for now: %s", print_ModuleStatus(last_running_module_status_).c_str());
    ROS_INFO("Trying to set module_status: %s", print_ModuleStatus(set_status).c_str());

    if (last_running_module_status_ == MODULE_IDLE){
        make_slam_obj(slam_param_, set_status);
        // mapping_status_ = M_STANDBY;
        // running_module_status_ = MODULE_MAPPING;
        return true;
    }else if(last_running_module_status_ == MODULE_MAPPING || 
             last_running_module_status_ == MODULE_SEC_MAPPING){
        ROS_INFO("skip, already running mapping now");//////////////////// TODO 要不要重置 start_index_ end_index_ ？？？
        return false;
    }else if (last_running_module_status_ == MODULE_LOCALIZATION){
        ROS_INFO("skip, running localizing now, please stop localizing first");
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("last_running_module_status_: %s", print_ModuleStatus(last_running_module_status_).c_str());
        ROS_INFO("set_module_status_: %s", print_ModuleStatus(set_module_status_).c_str());
        ROS_INFO("running_module_status_: %s", print_ModuleStatus(running_module_status_).c_str());
        ROS_INFO("mapping_status: %s", print_MappingStatus(mapping_status_).c_str());
        ROS_INFO("****************************");
        return false;
    }

}


bool LocalizationModule::start_second_mapping(ModuleStatus set_status, int map_id){
    ROS_INFO("Module Status for now: %s", print_ModuleStatus(last_running_module_status_).c_str());
    ROS_INFO("Trying to set module_status: %s", print_ModuleStatus(set_status).c_str());

    const auto use_ele_pcd_flag = slam_param_.mapping.use_ele_pcd_flag;
    std::string load_map_dir;
    if (use_ele_pcd_flag){
        load_map_dir = slam_param_.common.map_directory +std::string("/")+std::to_string(map_id)+std::string("/");
    }else{
        load_map_dir = slam_param_.common.map_directory +std::string("/");
    }


    if (last_running_module_status_ == MODULE_IDLE){
        make_slam_obj(slam_param_, set_status);
        // 加载地图
        ROS_INFO("load map dir: %s", load_map_dir.c_str());// 这种方式打印中文字符会乱码，显示为一堆问号，std::cout 可以正常打印中文
        slam_ -> load_map(load_map_dir);
        mapping_status_ = M_STANDBY;
        // running_module_status_ = MODULE_SEC_MAPPING;
        return true;
    }else if(last_running_module_status_ == MODULE_MAPPING || 
             last_running_module_status_ == MODULE_SEC_MAPPING){
        ROS_INFO("skip, already running mapping now");
        return false;
    }else if (last_running_module_status_ == MODULE_LOCALIZATION){
        ROS_INFO("skip, running localizing now, please stop localizing first");
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("last_running_module_status_: %s", print_ModuleStatus(last_running_module_status_).c_str());
        ROS_INFO("set_module_status_: %s", print_ModuleStatus(set_module_status_).c_str());
        ROS_INFO("running_module_status_: %s", print_ModuleStatus(running_module_status_).c_str());
        ROS_INFO("mapping_status: %s", print_MappingStatus(mapping_status_).c_str());
        ROS_INFO("****************************");
        return false;
    }
}

bool LocalizationModule::mark_start_point(){
    if (running_module_status_==MODULE_MAPPING || running_module_status_==MODULE_SEC_MAPPING){
        if(mapping_status_ == M_STANDBY){
            start_index_ = slam_->get_curr_pose_index();
            end_index_ = -1;
            ROS_INFO("start_index: %d", start_index_);
            ROS_INFO("end_index  : %d", end_index_);
            ROS_INFO("mapping_status_: %s", print_MappingStatus(mapping_status_).c_str());
            return true;
        }else{
            ROS_INFO("mapping_status_: %s", print_MappingStatus(mapping_status_).c_str());
            ROS_INFO("skip, please make sure mapping_status_ == M_STANDBY");
            return false;
        }
    }else if (running_module_status_==MODULE_IDLE || running_module_status_ == MODULE_LOCALIZATION){
        ROS_INFO("skip, running module status: %s ", print_ModuleStatus(running_module_status_).c_str());
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("set_module_status_: %s", print_ModuleStatus(set_module_status_).c_str());
        ROS_INFO("running_module_status_: %s", print_ModuleStatus(running_module_status_).c_str());
        ROS_INFO("mapping_status_: %s", print_MappingStatus(mapping_status_).c_str());
        ROS_INFO("localization_status_: %s", print_LocalizationStatus(localization_status_).c_str());
        ROS_INFO("****************************");
        return false;
    }
}


bool LocalizationModule::mark_end_point(int save_id){
    if (running_module_status_==MODULE_MAPPING || running_module_status_==MODULE_SEC_MAPPING){
        if(mapping_status_ == M_CREATING_ELE){
            end_index_ = slam_->get_curr_pose_index();
            ROS_INFO("start_index: %d", start_index_);
            ROS_INFO("end_index  : %d", end_index_);
            ROS_INFO("mapping_status_: %s", print_MappingStatus(mapping_status_).c_str());

            const auto save_ele_pcd_flag = slam_param_.mapping.save_ele_pcd_flag;
            if(save_ele_pcd_flag){
                // save pcd
                std::string pcd_path = slam_param_.common.map_directory +std::string("/")+ std::to_string(save_id)+std::string("/");
                if (save_id > 0){ // save_id == 0, 表示是禁区， 不保存小的 pcd
                    ROS_INFO("saving cloud map of current element ...");
                    const auto resolution = slam_param_.mapping.save_map_resolution;
                    slam_->save_map(pcd_path, resolution, start_index_, end_index_);
                }
            }else{
                ROS_INFO("Set to not saving ele pcd");
            }// 
            return true;

        }else{
            ROS_INFO("mapping_status_: %s", print_MappingStatus(mapping_status_).c_str());
            ROS_INFO("skip, please make sure mapping_status_ == M_STANDBY");
            return false;
        }
    }else if (running_module_status_==MODULE_IDLE || running_module_status_ == MODULE_LOCALIZATION){
        ROS_INFO("skip, running module status: %s ", print_ModuleStatus(running_module_status_).c_str());
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("set_module_status_: %s", print_ModuleStatus(set_module_status_).c_str());
        ROS_INFO("running_module_status_: %s", print_ModuleStatus(running_module_status_).c_str());
        ROS_INFO("mapping_status_: %s", print_MappingStatus(mapping_status_).c_str());
        ROS_INFO("localization_status_: %s", print_LocalizationStatus(localization_status_).c_str());
        ROS_INFO("****************************");
        return false;
    }

}


bool LocalizationModule::clear_curr_element(){
    if (running_module_status_==MODULE_MAPPING || running_module_status_==MODULE_SEC_MAPPING){
        if (mapping_status_ == M_CREATING_ELE){
            start_index_ = -1;
            end_index_ = -1;
            ROS_INFO("reset start & end point!");
            ROS_INFO("start_index: %d", start_index_);
            ROS_INFO("end_index  : %d", end_index_);
            return true;
        }else{
            ROS_INFO("mapping_status_: %s", print_MappingStatus(mapping_status_).c_str());
            ROS_INFO("skip, please make sure mapping_status_ == M_CREATING_ELE");
            return false;
        }
    }else if (running_module_status_==MODULE_IDLE || running_module_status_ == MODULE_LOCALIZATION){
        ROS_INFO("skip, running module status: %s ", print_ModuleStatus(running_module_status_).c_str());
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("set_module_status_: %s", print_ModuleStatus(set_module_status_).c_str());
        ROS_INFO("running_module_status_: %s", print_ModuleStatus(running_module_status_).c_str());
        ROS_INFO("mapping_status_: %s", print_MappingStatus(mapping_status_).c_str());
        ROS_INFO("localization_status_: %s", print_LocalizationStatus(localization_status_).c_str());
        ROS_INFO("****************************");
        return false;
    }
}


bool LocalizationModule::stop_mapping(){
    // TODO: 问题：当点云量为0时，程序挂掉
    if (last_running_module_status_ == MODULE_MAPPING || last_running_module_status_ == MODULE_SEC_MAPPING){
        if(mapping_status_ == M_STANDBY){
            ROS_INFO("mapping_status: %s", print_MappingStatus(mapping_status_).c_str());
            ROS_INFO("saving global map");
            std::string pcd_dir = slam_param_.common.map_directory;
            const auto resolution = slam_param_.mapping.save_map_resolution;
            slam_->save_map(pcd_dir, resolution, 0, 0);

            ROS_INFO("start stop mapping");
            // mapping_status_ = M_INACTIVE;
            sleep(1);
            // set_module_status_ = MODULE_IDLE;
            release_slam_obj();
            ROS_INFO("mapping stopped !");
            return true;
        }else if(mapping_status_ == M_CREATING_ELE){
            ROS_INFO("mapping_status: %s", print_MappingStatus(mapping_status_).c_str());
            ROS_INFO("skip, please finish current map-element, or delete it first !");
            return false;
        }else{
            ROS_INFO("skip, status error!");
            ROS_INFO("set_module_status_: %s", print_ModuleStatus(set_module_status_).c_str());
            ROS_INFO("running_module_status_: %s", print_ModuleStatus(running_module_status_).c_str());
            ROS_INFO("mapping_status_: %s", print_MappingStatus(mapping_status_).c_str());
            ROS_INFO("localization_status_: %s", print_LocalizationStatus(localization_status_).c_str());
            ROS_INFO("****************************");
            return false;
        }
    }else if(last_running_module_status_==MODULE_IDLE || last_running_module_status_==MODULE_LOCALIZATION){
        ROS_INFO("skip, can not stop mapping, running_module_status_: %s", print_ModuleStatus(running_module_status_).c_str());
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("last_running_module_status_: %s", print_ModuleStatus(last_running_module_status_).c_str());
        ROS_INFO("set_module_status_: %s", print_ModuleStatus(set_module_status_).c_str());
        ROS_INFO("running_module_status_: %s", print_ModuleStatus(running_module_status_).c_str());
        ROS_INFO("mapping_status_: %s", print_MappingStatus(mapping_status_).c_str());
        ROS_INFO("localization_status_: %s", print_LocalizationStatus(localization_status_).c_str());
        ROS_INFO("****************************");
        return false;
    }

    // running_module_status_ = MODULE_IDLE;

    // return;
}


bool LocalizationModule::start_localization(ModuleStatus set_status, int map_id){
    ROS_INFO("Module Status for now: %s", print_ModuleStatus(last_running_module_status_).c_str());
    ROS_INFO("Trying to set module_status: %s", print_ModuleStatus(set_status).c_str());

    const auto use_ele_pcd_flag = slam_param_.mapping.use_ele_pcd_flag;
    std::string load_map_dir = "";
    if (use_ele_pcd_flag){
        load_map_dir = slam_param_.common.map_directory +std::string("/")+std::to_string(map_id)+std::string("/");
    }else{
        load_map_dir = slam_param_.common.map_directory +std::string("/");
    }


    if (last_running_module_status_ == MODULE_IDLE){
        make_slam_obj(slam_param_, set_status);
        // 加载地图
        ROS_INFO("load map dir: %s", load_map_dir.c_str());
        slam_ -> load_map(load_map_dir);
        return true;
    }else if (last_running_module_status_ == MODULE_LOCALIZATION){
        ROS_INFO("skip, already running mapping now");
        return false;
    }else if(last_running_module_status_ == MODULE_MAPPING || 
             last_running_module_status_ == MODULE_SEC_MAPPING){
        ROS_INFO("skip, running localizing now, please stop localizing first");
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("last_running_module_status_: %s", print_ModuleStatus(last_running_module_status_).c_str());
        ROS_INFO("set_module_status_: %s", print_ModuleStatus(set_module_status_).c_str());
        ROS_INFO("running_module_status_: %s", print_ModuleStatus(running_module_status_).c_str());
        ROS_INFO("mapping_status: %s", print_MappingStatus(mapping_status_).c_str());
        ROS_INFO("****************************");
        return false;
    }
}

bool LocalizationModule::stop_localization(){

    if (last_running_module_status_==MODULE_LOCALIZATION){
        sleep(1);
        release_slam_obj();
        ROS_INFO("localization stopped !");
        // running_module_status_ = MODULE_IDLE;
        return true;

    }else if(last_running_module_status_==MODULE_IDLE || 
                last_running_module_status_==MODULE_MAPPING|| 
                last_running_module_status_==MODULE_SEC_MAPPING){
        ROS_INFO("skip, can not stop localization, running_module_status_: %s", print_ModuleStatus(running_module_status_).c_str());
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("last_running_module_status_: %s", print_ModuleStatus(last_running_module_status_).c_str());
        ROS_INFO("set_module_status_: %s", print_ModuleStatus(set_module_status_).c_str());
        ROS_INFO("running_module_status_: %s", print_ModuleStatus(running_module_status_).c_str());
        ROS_INFO("mapping_status_: %s", print_MappingStatus(mapping_status_).c_str());
        ROS_INFO("localization_status_: %s", print_LocalizationStatus(localization_status_).c_str());
        ROS_INFO("****************************");
        return false;
    }
}


bool LocalizationModule::make_slam_obj(lidar_slam::LidarSlamParam yaml_param, ModuleStatus set_status){
    ROS_INFO("creating lidar_slam ");
    lidar_slam::SlamWorkMode set_slam_mode = lidar_slam::SlamWorkMode::UNKNOWN;
    if(set_status == MODULE_MAPPING){
        set_slam_mode = lidar_slam::SlamWorkMode::MAPPING;
    }else if(set_status == MODULE_SEC_MAPPING){
        set_slam_mode = lidar_slam::SlamWorkMode::SEC_MAPPING;
    }else if(set_status == MODULE_LOCALIZATION){
        set_slam_mode = lidar_slam::SlamWorkMode::LOCALIZATION;
    }else{
        ROS_ERROR("ModuleStatus: %s, status error!", print_ModuleStatus(set_status).c_str());
        return false;
    }
    ROS_INFO("set slam work mode: %s", lidar_slam::print_SlamWorkMode(set_slam_mode).c_str());
    slam_ = std::make_unique<lidar_slam::LidarSlam>(yaml_param, set_slam_mode);
    return true;
}

void LocalizationModule::release_slam_obj(){
    if (set_module_status_ != MODULE_IDLE){
        ROS_INFO("skip, set module status(== %s) must be MODULE_IDLE before release slam obj", print_ModuleStatus(set_module_status_).c_str());
        return;
    }
    sleep(1);
    ROS_INFO("start stopping lidar_slam");
    lidar_slam::LidarSlam *temp_slam = slam_.release();
    ROS_INFO("release successfully");
    delete temp_slam;
    temp_slam = nullptr;
    ROS_INFO("delete successfully");

    start_index_ = -1;
    end_index_ = -1;
    running_module_status_ = MODULE_IDLE;
    mapping_status_ = M_INACTIVE;
    localization_status_ = L_INACTIVE;
    ROS_INFO("lidar_slam stopped !");
}

void LocalizationModule::slam_dealt_timer(const ros::TimerEvent &event){
    // SLAM 主要流程， 对应于原来的 while (ros::ok()){...}

    // if (!running_slam_){
    if (running_module_status_ == MODULE_IDLE || 
        running_module_status_ == MODULE_STARTING_SLAM || 
        running_module_status_ == MODULE_STOPPING_SLAM){
        ROS_INFO("running module status: IDLE (not running slam)");
        sleep(1);
        return;
    }
    if(set_module_status_ == MODULE_IDLE){
        ROS_INFO("setting module to IDLE ------------------------");
        sleep(3);
        return;
    }


    // if(second_mapping_ && !slam_->isGloalLocalizationSuccess()){
    //     ROS_INFO("processing GloalLocalization ...");
    //     return;
    // }

    // ROS_INFO("***********************************************");
    // cout<<"-----------------------------------------------"<<endl;
    // cout<<"***********************************************"<<endl;
    // ROS_INFO("running module status: %s", print_ModuleStatus(running_module_status_).c_str());

    if (control_status_.reset){
        sleep(1);
        slam_.reset();
        sleep(1);
        localization_mode_ = control_status_.localizationMode;
        // slam_ = std::make_unique<lidar_slam::LidarSlam>(curr_dir_+std::string("/lib/"),localization_mode_,offline_mode_);
        slam_ = std::make_unique<lidar_slam::LidarSlam>(curr_dir_+std::string("/"),localization_mode_,offline_mode_, 0);
        show_load_map_ = 0;
        control_status_.reset = false;

    }
    
    // ROS_INFO("trying to get loaded map...");
    // if (show_load_map_==0 && localization_mode_ && (slam_->getLoadMap())->points.size() > 0){
    // if (show_load_map_==0 && localization_mode_ && (slam_->getLoadMap()) && (slam_->getLoadMap())->points.size() > 0){
    if (show_load_map_==0 && running_module_status_==MODULE_LOCALIZATION && (slam_->getLoadMap()) && (slam_->getLoadMap())->points.size() > 0){
        // ROS_INFO("load map");
        sleep(1);
        sensor_msgs::PointCloud2 loadMap;
        pcl::toROSMsg(*(slam_->getLoadMap()), loadMap); 
        loadMap.header.stamp = ros::Time::now();
        loadMap.header.frame_id = "map";
        pubLoadMap.publish(loadMap);
        show_keyframe(slam_->getLoadKeyFrame(), pubKeyframePose);
        ROS_INFO("load map success");
        show_load_map_ ++;
    }

    if (just_show_mode_){
        return;
    } 
    /********************************- run slam -********************************/    
    bool running_slam_flag = slam_->run();

    if (running_slam_flag && show_rviz_){
        // if (!localization_mode_ || slam_->isGloalLocalizationSuccess()){
        if ((running_module_status_ == MODULE_MAPPING || running_module_status_ == MODULE_SEC_MAPPING) 
            || slam_->isGloalLocalizationSuccess()){
            pub_odom_cloud(slam_->get_odom_cloud(), pubOdomCloud);
        }
        pub_test_cloud(slam_->getTestCloud(), localization_mode_, pubTestCloud);
        pub_lidar_cloud(slam_->get_lidar_cloud(), pubBodyCloud);
        pub_obstacle_cloud(slam_->getObstacleCloud(), pubObstacleCloud);
        pub_filtered_obstacle_cloud(slam_->getFilteredObstacleCloud(), pubFilteredObstacleCloud);
        publish_unoptimized_path(slam_->get_unoptimized_path(),pubUnoptimizedPath);
        publish_optimized_path(slam_->get_optimized_path(),string("odom"), pubOptimizedPath);
        visualizeLoopClosure(slam_->getloopIndex(),optimized_path_msg, pubLoopConstraintEdge);
        publish_transform(slam_->getOdomToMap(),string("map"),string("odom"));
        // pub_kdtree_cloud(slam_->get_kdtree_cloud());		//not used yet
    }
    // if(!localization_mode_){
    if(running_module_status_ == MODULE_MAPPING || running_module_status_ == MODULE_SEC_MAPPING){
    // pub_rgb_map(slam->getCurrentRGBMap());
        publish_odometry_lidar_in_map(slam_->getLidarInMap(), "map", "base_footprint", pubLidarInMap);
    }else{
        publish_odometry_lidar_in_map(slam_->getLidarInMap(), "map", "base_footprint", pubLidarInMap);
        publish_odometry(slam_->getLidarInOdom(), pubOdomAftMapped);
        pub_lidar_cloud(slam_->get_lidar_cloud(), pubBodyCloud);
    }
    if (show_rviz_){
        publish_static_transform(slam_->getWheelInLidar());
        publish_odometry(slam_->getLidarInOdom(), pubOdomAftMapped);
    }
}



// void LocalizationModule::livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg_in){
void LocalizationModule::livox_pcl_cbk(const fairland_msgs::LivoxCustomMsg::ConstPtr &msg_in){
    // if (!running_slam_){
    if (set_module_status_ == MODULE_IDLE || running_module_status_ == MODULE_IDLE){
        return;
    }
    if(control_status_.reset||offline_mode_)
       return;
	std::shared_ptr<livox_ros::LidarMsg> msg(new livox_ros::LidarMsg);
	msg->time_stamp = msg_in->header.stamp.toSec();
    msg->point_num = msg_in->point_num;
   // msg->lidar_id;
  //  msg->rsvd[3];
    msg->points.resize(msg_in->points.size());
    for (int i =0; i<msg_in->points.size(); i++){
		msg->points[i].x = msg_in->points[i].x;
        msg->points[i].y = msg_in->points[i].y;
        msg->points[i].z = msg_in->points[i].z;
        msg->points[i].reflectivity = msg_in->points[i].reflectivity;
        msg->points[i].offset_time = msg_in->points[i].offset_time;
		msg->points[i].tag = msg_in->points[i].tag;
        msg->points[i].line = msg_in->points[i].line;
	}
	slam_ -> livox_pcl_cbk(msg);
    
}


void LocalizationModule::livox_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &ros_msg){
    // ROS_INFO("livox lidar callback~");
    // if (!running_slam_){
    if (set_module_status_ == MODULE_IDLE || running_module_status_ == MODULE_IDLE){
        return;
    }

    if(control_status_.reset||offline_mode_)
       return;
	
    int cloud_num = ros_msg->height * ros_msg->width;
    
    ///// MetaData --- header 
    pcl::PCLHeader pcl_header;
    pcl_header.seq = ros_msg->header.seq;
    pcl_header.stamp = ros_msg->header.stamp.toNSec() / 1000ull;
    pcl_header.frame_id = ros_msg->header.frame_id;
    ///// MetaData --- field
    std::vector<pcl::PCLPointField> pcl_fields;
    pcl_fields.resize(ros_msg->fields.size());
    std::vector<sensor_msgs::PointField>::const_iterator it = ros_msg->fields.begin();
    int i = 0;
    for(; it != ros_msg->fields.end(); ++it, ++i) {
      pcl_fields[i].name = it->name;
      pcl_fields[i].offset = it->offset;
      pcl_fields[i].datatype = it->datatype;
      pcl_fields[i].count = it->count;
    }
    //// create Mapping
    pcl::MsgFieldMap field_map;
    pcl::createMapping<LvxPointXYZITLO> (pcl_fields, field_map);

    std::shared_ptr<livox_ros::LidarMsg> msg(new livox_ros::LidarMsg);
    // msg->points.resize(cloud_num);

    for (std::uint32_t row = 0; row < ros_msg->height; ++row){
        const std::uint8_t* row_data = &ros_msg->data[row * ros_msg->row_step];
        for (std::uint32_t col = 0; col < ros_msg->width; ++col){
            const std::uint8_t* msg_data = row_data + col * ros_msg->point_step;
            LvxPointXYZITLO temp_point;
            LvxPointXYZITLO* curpt = &temp_point;
            std::uint8_t* curpt_data = reinterpret_cast<std::uint8_t*>(curpt);

            for (const pcl::detail::FieldMapping& mapping : field_map){
                memcpy (curpt_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
            }
            livox_ros::LidarPoint livox_point;
            livox_point.x = curpt->x;
            livox_point.y = curpt->y;
            livox_point.z = curpt->z;
            livox_point.reflectivity = curpt->intensity;
            livox_point.tag = curpt->tag;
            livox_point.line = curpt->line;
            livox_point.offset_time = curpt->offset_time;
            msg->points.push_back(livox_point);
        }
    }

    msg->time_stamp = ros_msg->header.stamp.toSec();
    msg->point_num = cloud_num;

    // if (running_slam_){
    //     slam_ -> livox_pcl_cbk(msg);
    // }

    if (set_module_status_ == MODULE_IDLE || running_module_status_ == MODULE_IDLE){
        return;
    }else{
        slam_ -> livox_pcl_cbk(msg);
        // ROS_INFO("lidar callback success");
        return;
    }

}

void LocalizationModule::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in){
    // ROS_INFO("livox imu callback~");
    // if (!running_slam_){
    if (set_module_status_ == MODULE_IDLE || running_module_status_ == MODULE_IDLE){
        return;
    }
    if(control_status_.reset||offline_mode_)
       return;

    std::shared_ptr<livox_ros::ImuMsg> msg(new livox_ros::ImuMsg);
	msg->time_stamp = msg_in->header.stamp.toSec();
	msg->angular_velocity << msg_in->angular_velocity.x,msg_in->angular_velocity.y,msg_in->angular_velocity.z;
	msg->linear_acceleration << msg_in->linear_acceleration.x,msg_in->linear_acceleration.y,msg_in->linear_acceleration.z;	

    // if (running_slam_){
    //     slam_ -> imu_cbk(msg);
    // }

    if (set_module_status_ == MODULE_IDLE || running_module_status_ == MODULE_IDLE){
        return;
    }else{
        slam_ -> imu_cbk(msg);
        return;
    }

}

void LocalizationModule::show_thread()
{
    std::cout<<"start show thread "<<endl;
    const int frequency = 5.0; // 频率为1Hz
    const std::chrono::milliseconds period(1000 / frequency);
    lidar_slam::Viewer test_view(localization_mode_);
    while (ros::ok())
    {
        auto start = std::chrono::steady_clock::now();
        if (!control_status_.reset){
            test_view.Start();
            control_status_ = test_view.getControl();
            if (!localization_mode_){
                std::vector<Eigen::Isometry3d> optimized_poses = slam_->get_optimized_path();
                test_view.DrawTrajectory(optimized_poses,Eigen::Vector3f(0,1,0));
                test_view.DrawTrajectory(slam_->get_unoptimized_path(),Eigen::Vector3f(1,0,0));
                map<int, int> loopIndex = slam_->getloopIndex();
                for (auto it = loopIndex.begin(); it != loopIndex.end(); ++it) {
                     test_view.DrawLine(optimized_poses[it->first],optimized_poses[it->second],Eigen::Vector3f(0,0,0));
                }
                
                if (control_status_.showMap)
                    test_view.DrawCloud(slam_->getCurrentMap(),Eigen::Vector3f(0,0,1),1);
                if (control_status_.showLidar)
                   test_view.DrawCloud(slam_->get_odom_cloud(),slam_->getOdomToMap(),Eigen::Vector3f(1,0,0),2);
                if (control_status_.showObstacle)
                    test_view.DrawCloud(slam_->getFilteredObstacleCloud(),slam_->getWheelInMap(),Eigen::Vector3f(0,1,0),2.0);
                test_view.DrawPose(slam_->getWheelInMap());
            }
            else{
                if (control_status_.showMap)
                   test_view.DrawCloud(slam_->getLoadMapPoints(),Eigen::Vector3f(0,0,1),1.0);
                if (control_status_.showLidar)
                   test_view.DrawCloud(slam_->get_lidar_cloud(),slam_->getLidarInMap(),Eigen::Vector3f(1,0,0),2.0);
                if (control_status_.showObstacle)
                    test_view.DrawCloud(slam_->getFilteredObstacleCloud(),slam_->getWheelInMap(),Eigen::Vector3f(0,1,0),2.0);
                if (slam_->isGloalLocalizationSuccess())
                    test_view.DrawPose(slam_->getWheelInMap());
                test_view.DrawTrajectory(slam_->get_unoptimized_path(),Eigen::Vector3f(1,0,0));
            }

            if (control_status_.saveMap && !localization_mode_)
                slam_ -> save_map(curr_dir_+std::string("/map/"),0.1, 0, 0);
            test_view.Finish();  
        }
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        if (elapsed < period)
        {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}


// 这里其实还包含了 update path
void LocalizationModule::publish_unoptimized_path(const std::deque<Eigen::Isometry3d> path, ros::Publisher pubUnoptimizedPath)
{
	geometry_msgs::PoseStamped msg;

	int size = path.size();
	unoptimized_path_msg.poses.clear();
    unoptimized_path_msg.header.stamp = ros::Time().now();
    unoptimized_path_msg.header.frame_id = "odom";
	for (int i = 0 ; i <path.size();i++){
		msg.header.stamp = ros::Time().now();
		msg.header.frame_id = "odom";
		msg.pose.position.x = path[i].translation().x();
		msg.pose.position.y = path[i].translation().y();
		msg.pose.position.z = path[i].translation().z();
		/*Eigen::Quaterniond quaternion = path[i].rotation();
		msg.pose.orientation.x = quaternion.x();
		msg.pose.orientation.y = quaternion.y();
		msg.pose.orientation.z = quaternion.z();
		msg.pose.orientation.w = quaternion.w();*/
		unoptimized_path_msg.poses.push_back(msg);
	}
    pubUnoptimizedPath.publish(unoptimized_path_msg);
}

void LocalizationModule::publish_optimized_path(const std::vector<Eigen::Isometry3d> path,std::string frame, ros::Publisher pubOptimizedPath)
{
	geometry_msgs::PoseStamped msg;
    optimized_path_msg.poses.clear();
    optimized_path_msg.header.stamp = ros::Time().now();
    optimized_path_msg.header.frame_id = frame;
	int size = path.size();
	for (int i = 0 ; i <path.size();i++){
		msg.header.stamp = ros::Time().now();
		msg.header.frame_id = frame;
		msg.pose.position.x = path[i].translation().x();
		msg.pose.position.y = path[i].translation().y();
		msg.pose.position.z = path[i].translation().z();
		/*Eigen::Quaterniond quaternion = path[i].rotation();
		msg.pose.orientation.x = quaternion.x();
		msg.pose.orientation.y = quaternion.y();localization_mode_
		msg.pose.orientation.z = quaternion.z();
		msg.pose.orientation.w = quaternion.w();*/
		optimized_path_msg.poses.push_back(msg);
	}
    pubOptimizedPath.publish(optimized_path_msg);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////--------------------------- Init Module -----------------------------////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void LocalizationModule::load_params(){
    nh_.param<bool>("localization_mode", localization_mode_, false);
    nh_.param<bool>("just_show_mode", just_show_mode_, false);
    nh_.param<bool>("offline_mode_", offline_mode_, false);
    nh_.param<bool>("show_rviz", show_rviz_, false);
    // nh_.param<bool>("fast", fast_mode_, false);
    // nh_.param<string>("log_folder", log_folder_, " ");

}


bool LocalizationModule::load_lidar_slam_param(){
    std::string ns = "/flbot/lidar_slam/";
    bool success = true;
    /// common *******************************************
    get_param(ns+ "common/time_sync_en", slam_param_.common.time_sync_en, &success);
    get_param(ns+ "common/localization_mode", slam_param_.common.localization_mode, &success);
    get_param(ns+ "common/offline_mode", slam_param_.common.offline_mode, &success);
    get_param(ns+ "common/fast_mode", slam_param_.common.fast_mode, &success);
    get_param(ns+ "common/just_show_mode", slam_param_.common.just_show_mode, &success);
    get_param(ns+ "common/show_rviz", slam_param_.common.show_rviz, &success);
    get_param(ns+ "common/save_log_dir", slam_param_.common.save_log_dir, &success);
    get_param(ns+ "common/log_keep_time", slam_param_.common.log_keep_time, &success);
    get_param(ns+ "common/map_relative_to", slam_param_.common.map_relative_to, &success);
    get_param(ns+ "common/map_directory", slam_param_.common.map_directory, &success);
    get_param(ns+ "common/sub_topic_ctrl_cmd", slam_param_.common.sub_topic_ctrl_cmd, &success);
    get_param(ns+ "common/pub_topic_module_status", slam_param_.common.pub_topic_module_status, &success);

    // process map_dir
    std::cout << "C++ Standard: " << __cplusplus << std::endl;
    std::string parent_dir;
    if(slam_param_.common.map_relative_to == 0){//相对于 pkg
        std::string package_path = ros::package::getPath("lidar_slam");
        parent_dir = package_path;
    }else if(slam_param_.common.map_relative_to == 1){//相对于catkin_ws
        std::filesystem::path package_path = ros::package::getPath("lidar_slam");
        parent_dir = package_path.parent_path().parent_path().generic_string();
    }else if(slam_param_.common.map_relative_to == 2){//绝对路径
        parent_dir="";
    }

    slam_param_.common.map_directory = parent_dir + slam_param_.common.map_directory;

    /// extrinsic *******************************************
    std::vector<double> extrinsic_T; // 1 * 3
    std::vector<double> extrinsic_R; // 3 * 3
    std::vector<double> Lidar_In_Wheel; // 4* 4
    get_param(ns+ "extrinsic/extrinsic_est_en", slam_param_.extrinsic.extrinsic_est_en, &success);
    get_param(ns+ "extrinsic/extrinsic_T", extrinsic_T, &success);//temp
    get_param(ns+ "extrinsic/extrinsic_R", extrinsic_R, &success);//temp
    get_param(ns+ "extrinsic/Lidar_In_Wheel", Lidar_In_Wheel, &success);//temp
   
    // extrinT & extrinR
    slam_param_.extrinsic.extrinT<< extrinsic_T[0],extrinsic_T[1],extrinsic_T[2];
    slam_param_.extrinsic.extrinR<< extrinsic_R[0],extrinsic_R[1],extrinsic_R[2],
                                    extrinsic_R[3],extrinsic_R[4],extrinsic_R[5],
                                    extrinsic_R[6],extrinsic_R[7],extrinsic_R[8];
    // T_wheel_lidar & T_lidar_wheel
    Eigen::Matrix4d T_wheel_lidar;
    T_wheel_lidar<< Lidar_In_Wheel[0], Lidar_In_Wheel[1], Lidar_In_Wheel[2], Lidar_In_Wheel[3],
                    Lidar_In_Wheel[4], Lidar_In_Wheel[5], Lidar_In_Wheel[6], Lidar_In_Wheel[7],
                    Lidar_In_Wheel[8], Lidar_In_Wheel[9], Lidar_In_Wheel[10],Lidar_In_Wheel[11],
                    Lidar_In_Wheel[12],Lidar_In_Wheel[13],Lidar_In_Wheel[14],Lidar_In_Wheel[15];
    slam_param_.extrinsic.T_wheel_lidar.matrix() = T_wheel_lidar;
    slam_param_.extrinsic.T_lidar_wheel = slam_param_.extrinsic.T_wheel_lidar.inverse();

    /// lidar_preproc params *******************************************
    get_param(ns+ "lidar_preproc/line_count", slam_param_.lidar_preproc.line_count, &success);
    get_param(ns+ "lidar_preproc/blind_distance", slam_param_.lidar_preproc.blind_distance, &success);
    get_param(ns+ "lidar_preproc/point_filter_num", slam_param_.lidar_preproc.point_filter_num, &success);
    get_param(ns+ "lidar_preproc/feature_enabled", slam_param_.lidar_preproc.feature_enabled, &success);
    get_param(ns+ "lidar_preproc/obstacle_max_range", slam_param_.lidar_preproc.obstacle_max_range, &success);
    get_param(ns+ "lidar_preproc/obstacle_max_height", slam_param_.lidar_preproc.obstacle_max_height, &success);
    get_param(ns+ "lidar_preproc/obstacle_min_height", slam_param_.lidar_preproc.obstacle_min_height, &success);
    get_param(ns+ "lidar_preproc/obstacle_filter_size", slam_param_.lidar_preproc.obstacle_filter_size, &success);
    get_param(ns+ "lidar_preproc/grid_size", slam_param_.lidar_preproc.grid_size, &success);

    /// mapping params *******************************************
    get_param(ns+ "mapping/acc_cov", slam_param_.mapping.acc_cov, &success);
    get_param(ns+ "mapping/gyr_cov", slam_param_.mapping.gyr_cov, &success);
    get_param(ns+ "mapping/b_acc_cov", slam_param_.mapping.b_acc_cov, &success);
    get_param(ns+ "mapping/b_gyr_cov", slam_param_.mapping.b_gyr_cov, &success);
    get_param(ns+ "mapping/cloud_leaf_size", slam_param_.mapping.cloud_leaf_size, &success);
    get_param(ns+ "mapping/key_frame_distance", slam_param_.mapping.key_frame_distance, &success);
    get_param(ns+ "mapping/key_frame_angle", slam_param_.mapping.key_frame_angle, &success);
    get_param(ns+ "mapping/loopSearchDistance", slam_param_.mapping.loopSearchDistance, &success);
    get_param(ns+ "mapping/use_ele_pcd_flag", slam_param_.mapping.use_ele_pcd_flag, &success);
    get_param(ns+ "mapping/save_ele_pcd_flag", slam_param_.mapping.save_ele_pcd_flag, &success);
    // get_param(ns+ "mapping/save_map_dir", slam_param_.mapping.save_map_dir, &success);
    get_param(ns+ "mapping/save_map_resolution", slam_param_.mapping.save_map_resolution, &success);
    
    /// sec_mapping params *******************************************
    // get_param(ns+ "sec_mapping/load_map_dir", slam_param_.sec_mapping.load_map_dir, &success);


    /// localization params *******************************************
    // get_param(ns+ "localization/load_map_dir", slam_param_.localization.load_map_dir, &success);


    /// re-localization params *******************************************
    get_param(ns+ "re_localization/score_thr", slam_param_.re_localization.score_thr, &success);
    get_param(ns+ "re_localization/time_out_thr", slam_param_.re_localization.time_out_thr, &success);


    /// ikdtree params *******************************************
    get_param(ns+ "ikdtree/cube_len", slam_param_.ikdtree.cube_len, &success);
    get_param(ns+ "ikdtree/det_range", slam_param_.ikdtree.det_range, &success);
    get_param(ns+ "ikdtree/kdTreeReconstructRadius", slam_param_.ikdtree.kdTreeReconstructRadius, &success);
    get_param(ns+ "ikdtree/kdTreeReconstructKeyFrameLeafSize", slam_param_.ikdtree.kdTreeReconstructKeyFrameLeafSize, &success);
    get_param(ns+ "ikdtree/kdTreeReconstructPointLeafSize", slam_param_.ikdtree.kdTreeReconstructPointLeafSize, &success);
    get_param(ns+ "ikdtree/map_leaf_size", slam_param_.ikdtree.map_leaf_size, &success);

    return success;
}


void LocalizationModule::pub_module_status_timer(const ros::TimerEvent &event){
    // make msg *************************************************************************
    // fill header
    fairland_msgs::LocalizationModuleStatus status_msg;
    status_msg.header.stamp = ros::Time().now();
    status_msg.header.frame_id = "base_link";

    // fill status_msg.module_status
    if(running_module_status_ == MODULE_IDLE){
        status_msg.module_status = fairland_msgs::LocalizationModuleStatus::IDLE;
    }else if(running_module_status_ == MODULE_MAPPING || running_module_status_ == MODULE_SEC_MAPPING){
        status_msg.module_status = fairland_msgs::LocalizationModuleStatus::MAPPING;
    }else if(running_module_status_ == MODULE_LOCALIZATION){
        status_msg.module_status = fairland_msgs::LocalizationModuleStatus::LOCALIZATION;
    }else if(running_module_status_ == MODULE_STARTING_SLAM){
        status_msg.module_status = fairland_msgs::LocalizationModuleStatus::STARTING;
    }else if(running_module_status_ == MODULE_STOPPING_SLAM){
        status_msg.module_status = fairland_msgs::LocalizationModuleStatus::STOPPING;
    }else{
        ROS_ERROR("error running module status: %s", print_ModuleStatus(running_module_status_).c_str());
    }

    // fill status_msg.mapping_status
    if(mapping_status_ == M_INACTIVE){
        status_msg.mapping_status = fairland_msgs::LocalizationModuleStatus::M_INACTIVE;
    }else if(mapping_status_ == M_RELOCALIZING){
        status_msg.mapping_status = fairland_msgs::LocalizationModuleStatus::M_RELOCALIZING;
    }else if(mapping_status_ == M_RELOCALIZE_FAILED){
        status_msg.mapping_status = fairland_msgs::LocalizationModuleStatus::M_RELOCALIZE_FAILED;
    }else if(mapping_status_ == M_CREATING_ELE){
        status_msg.mapping_status = fairland_msgs::LocalizationModuleStatus::M_CREATING_ELE;
    }else if(mapping_status_ == M_STANDBY){
        status_msg.mapping_status = fairland_msgs::LocalizationModuleStatus::M_STANDBY;
    }else{
        ROS_ERROR("error mapping status: %s", print_MappingStatus(mapping_status_).c_str());
    }

    // fill status_msg.localization_status
    localization_status_ = slam_->get_l_status();
    if(localization_status_ == L_INACTIVE){
        status_msg.localization_status = fairland_msgs::LocalizationModuleStatus::L_INACTIVE;
    }else if(localization_status_ == L_RELOCALIZING){
        status_msg.localization_status = fairland_msgs::LocalizationModuleStatus::L_RELOCALIZING;
    }else if(localization_status_ == L_RELOCALIZE_FAILED){
        status_msg.localization_status = fairland_msgs::LocalizationModuleStatus::L_RELOCALIZE_FAILED;
    }else if(localization_status_ == L_NORMAL){
        status_msg.localization_status = fairland_msgs::LocalizationModuleStatus::M_CREATING_ELE;
    }else if(localization_status_ == L_LOW_ACCURACY){
        status_msg.localization_status = fairland_msgs::LocalizationModuleStatus::L_LOW_ACCURACY;
    }else if(localization_status_ == L_FAILED){
        status_msg.localization_status = fairland_msgs::LocalizationModuleStatus::L_FAILED;
    }else{
        ROS_ERROR("error localization status: %s", print_LocalizationStatus(localization_status_).c_str());
    }

    pub_localization_module_status_.publish(status_msg);

}

bool LocalizationModule::create_ROS_IO(){
    // subscriber ********************************************************************
	// ros::Subscriber sub_pcl = nh_.subscribe<livox_ros_driver2::CustomMsg>("/livox/lidar", 200000, &LocalizationModule::livox_pcl_cbk, this);
    sub_pointcloud2_ = nh_.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 10, &LocalizationModule::livox_pcl_cbk, this);
    sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("/livox/imu", 200000, &LocalizationModule::imu_cbk, this);

    sub_mapping_ctrl_ = nh_.subscribe(slam_param_.common.sub_topic_ctrl_cmd, 3 ,&LocalizationModule::localization_module_ctrl_cbk, this);
    
    // timer dealt ********************************************************************
    // 建图主要流程，timer 时间间隔需要调整，10hz? 100hz? 200hz?
    timer_slam_ = nh_.createTimer(ros::Duration(0.05), &LocalizationModule::slam_dealt_timer, this);
    timer_pub_module_status_ = nh_.createTimer(ros::Duration(0.1), &LocalizationModule::pub_module_status_timer, this);
    
    // publish ************************************************************************
    pub_localization_module_status_ = nh_.advertise<fairland_msgs::LocalizationModuleStatus>(slam_param_.common.pub_topic_module_status, 100); 

    // both 建图 & 定位
	pubLidarInMap = nh_.advertise<nav_msgs::Odometry>("/Odometry_lidar_in_map", 100000);

    // only 建图 


    // only 定位

    // publish TODO: 还需要区分哪些是建图或定位发布的
    pubOdomCloud = nh_.advertise<sensor_msgs::PointCloud2>("/odom_cloud", 100000);  
	pubBodyCloud = nh_.advertise<sensor_msgs::PointCloud2>("/body_cloud", 100000);
    pubObstacleCloud = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle_cloud", 100000);
    pubFilteredObstacleCloud = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_obstacle_cloud", 100000);
    pubTestCloud = nh_.advertise<sensor_msgs::PointCloud2>("/test_cloud", 100000);
	pubKdtreeCloud = nh_.advertise<sensor_msgs::PointCloud2>("/kdtree_cloud", 100000); 
	pubOptimizedPath= nh_.advertise<nav_msgs::Path>("/optimized_path", 1000);
    pubUnoptimizedPath= nh_.advertise<nav_msgs::Path>("/unoptimized_path", 1000);
	pubLoopConstraintEdge = nh_.advertise<visualization_msgs::MarkerArray>("/loop_closure_constraints", 1);
    pubKeyframePose = nh_.advertise<visualization_msgs::MarkerArray>("/key_frame_pose", 1);
	pubOdomAftMapped = nh_.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    pubLoadMap = nh_.advertise<sensor_msgs::PointCloud2>("/Load_map", 100000);
    pubRgbCloud= nh_.advertise<sensor_msgs::PointCloud2>("rgb_cloud", 1);
    // image_pub = nh_.advertise<sensor_msgs::Image>("fisheye_image", 1);

    return true;
}

bool LocalizationModule::init_module_by_set_status(ModuleStatus set_status){
    set_module_status_ = set_status;

    if(set_module_status_ == MODULE_IDLE){
        // ROS_INFO("init module status: %s", print_ModuleStatus(set_module_status_).c_str());
    }else if (set_module_status_ == MODULE_MAPPING){
        if(start_mapping(set_module_status_)){
            running_module_status_ = set_module_status_;
            mapping_status_ = M_STANDBY;
        }else{
            set_module_status_ = running_module_status_;
        }
    }else if (set_module_status_ == MODULE_SEC_MAPPING){
        // TODO
        int map_id = 0;/////////////// TODO
        if(start_second_mapping(set_module_status_, map_id)){
            running_module_status_ = MODULE_SEC_MAPPING;
            mapping_status_ = M_STANDBY;
        }else{
            set_module_status_ = running_module_status_;
        }
    }else if (set_module_status_ == MODULE_LOCALIZATION){
        // TODO
        int map_id = 0;/////////////// TODO
        if(start_localization(set_module_status_, map_id)){
            running_module_status_ = MODULE_LOCALIZATION;
            // localization_status_ = L_RELOCALIZING;
        }else{
            set_module_status_ = running_module_status_;
        }
    }

    // ROS_INFO("init module status: %s", print_ModuleStatus(running_module_status_).c_str());

    return true;
}


// void LocalizationModule::start_mapping(bool localization_mode){
//     // TODO: 启动建图前，需要确定哪些参数？？
//     // localization_mode_;
//     // offline_mode_;
//     // 初始位姿？
//     ROS_INFO("trying to create lidar_slam, localization_mode: %s", localization_mode);
//     std::string workpath = curr_dir_+std::string("/");
//     if (!running_slam_){// slam 未激活
//         ROS_INFO("slam not running now");
//         bool second_mapping = false;
//         make_slam_obj(workpath, localization_mode, offline_mode_, second_mapping);
//         running_slam_ = true;
//         localization_mode_ = 0;
//         mapping_status_ = MAPPING_STARTED;//////////////// TODO, 状态调整或细化？加入定位？
//         return;
//     }else if (running_slam_ && slam_mode_==MAPPING){// slam 已经启动，且状态为: mapping 
//         ROS_INFO("skip, already running mapping now");//////////////////// TODO 要不要重置 start_index_ end_index_ ？？？
//         // reset start & end point
//         // ROS_INFO("running mapping now, reset start and end point");
//         // start_index_ = -1;
//         // end_index_ = -1;
//         return;
//     }else if (running_slam_ && slam_mode_==LOCALIZATION){// slam 已经启动，但状态为: localization
//         ROS_INFO("skip, running localizing now, please stop localizing first");
//         // stop_localization();
//         // make_slam_obj(workpath, localization_mode, offline_mode_);
//         // mapping_status_ = MAPPING_STARTED;//////////////// TODO
//         return;
//     }else{
//         ROS_INFO("skip, slam status error!");
//         ROS_INFO("running_slam: %u", running_slam_);
//         ROS_INFO("slam_mode: %s", print_SlamMode(slam_mode_).c_str());
//         ROS_INFO("mapping_status: %s", print_MappingStatus(mapping_status_).c_str());
//         ROS_INFO("****************************");
//         return;
//     }
// }

// void LocalizationModule::start_second_mapping(bool localization_mode, int map_id){
//     // 重定位
//     ROS_INFO("trying to create lidar_slam, localization_mode: %u", localization_mode);
//     std::string workpath = curr_dir_+std::string("/");
//     std::string pcd_path = curr_dir_+std::string("/map/")+std::to_string(map_id)+std::string("/");
//     if(!running_slam_){
//         // slam
//         bool second_mapping  = true;
//         make_slam_obj(workpath, localization_mode, offline_mode_, second_mapping);
//         // 加载地图
//         ROS_INFO("load map dir: %s", pcd_path.c_str());
//         slam_ -> load_map(pcd_path);
//         second_mapping_ = true;
//         running_slam_ = true;
//         localization_mode_ = 1;
//         mapping_status_ = MAPPING_STARTED;
//     }else if(running_slam_ && slam_mode_ == MAPPING){	
//         ROS_INFO("skip, running mapping now, please stop mapping first!");
//     }else{
//         ROS_INFO("skip, slam status error!");
//         ROS_INFO("running_slam: %u", running_slam_);
//         ROS_INFO("slam_mode: %s", print_SlamMode(slam_mode_).c_str());
//         ROS_INFO("mapping_status: %s", print_MappingStatus(mapping_status_).c_str());
//         ROS_INFO("****************************");
//     }
// }

// void LocalizationModule::make_slam_obj(string work_path, bool localization_mode, bool offline_mode, bool sec_mapping){
//     ROS_INFO("creating lidar_slam ");
//     slam_ = std::make_unique<lidar_slam::LidarSlam>(work_path, localization_mode, offline_mode, sec_mapping);
//     start_index_ = -1;
//     end_index_ = -1;
//     localization_mode_ = localization_mode;
//     // running_slam_ = true; // 在 start_mapping 和 relocalization_localize 中修改状态
//     if(localization_mode){
//         slam_mode_ = LOCALIZATION;
//     }else{
//         slam_mode_ = MAPPING;
//         mapping_status_ = MAPPING_STARTED;
//     }
//     ROS_INFO("slam_mode: %s", print_SlamMode(slam_mode_).c_str());
//     ROS_INFO("create lidar_slam successfully");
// }

// void LocalizationModule::mark_start_point(){
//     // 标记起点的 POSE
//     if(running_slam_ && slam_mode_==MAPPING && 
//         (mapping_status_ == MAPPING_STARTED || mapping_status_ == ENDPOINT_SET)){
//         start_index_ = slam_->get_curr_pose_index();
//         end_index_ = -1;
//         ROS_INFO("start_index: %d", start_index_);
//         ROS_INFO("end_index  : %d", end_index_);
//         mapping_status_ = STARTPOINT_SET;
//     }else if(!running_slam_){
//         ROS_INFO("skip, not running slam !");
//     }else if(running_slam_ && slam_mode_==LOCALIZATION){
//         ROS_INFO("skip, running slam (mode: localization) !");
//     } else {
//         ROS_INFO("skip, slam status error!");
//         ROS_INFO("running_slam: %u", running_slam_);
//         ROS_INFO("slam_mode: %s", print_SlamMode(slam_mode_).c_str());
//         ROS_INFO("mapping_status: %s", print_MappingStatus(mapping_status_).c_str());
//         ROS_INFO("****************************");
//     }
// }

// void LocalizationModule::mark_end_point(int save_id){
//     // 当前地图元素完成，判断是否保存点云地图
//     if(running_slam_ && slam_mode_==MAPPING && mapping_status_ == STARTPOINT_SET){
//         end_index_ = slam_->get_curr_pose_index();
//         mapping_status_ = ENDPOINT_SET;
//         ROS_INFO("start_index: %d", start_index_);
//         ROS_INFO("end_index  : %d", end_index_);
//         std::string pcd_path = curr_dir_ + std::string("/map/") + std::to_string(save_id)+std::string("/");
//         if (save_id > 0){ // save_id == 0, 表示是禁区， 不保存小的 pcd
//             ROS_INFO("saving cloud map of current element ...");
//             slam_->save_map(pcd_path, 0.1, start_index_, end_index_);
//         }
//     }else if(running_slam_ && slam_mode_==MAPPING && mapping_status_ != STARTPOINT_SET){
//         ROS_INFO("mapping_status: %s", print_MappingStatus(mapping_status_).c_str());
//         ROS_INFO("skip, 'mapping_status==STARTPOINT_SET' required, please set start-point first!");
//     }else{
//         ROS_INFO("skip, slam status error!");
//         ROS_INFO("running_slam: %u", running_slam_);
//         ROS_INFO("slam_mode: %s", print_SlamMode(slam_mode_).c_str());
//         ROS_INFO("mapping_status: %s", print_MappingStatus(mapping_status_).c_str());
//         ROS_INFO("****************************");
//     }
// }

// void LocalizationModule::clear_curr_element(){
//     // 清除当前正在创建的元素，清除 标记的Pose
//     if(running_slam_ && (slam_mode_==MAPPING) && (mapping_status_ == STARTPOINT_SET)){
//         start_index_ = -1;
//         end_index_ = -1;
//         mapping_status_ = MAPPING_STARTED;
//         ROS_INFO("reset start & end point!");
//         ROS_INFO("start_index: %d", start_index_);
//         ROS_INFO("end_index  : %d", end_index_);
//         return;
//     }else if (!running_slam_){
//         ROS_INFO("skip, not running slam !");
//         return;
//     }else if(running_slam_ && (slam_mode_ == LOCALIZATION)){
//         ROS_INFO("skip, running slam (mode: localization) !");
//         return;
//     }else if(running_slam_ && (slam_mode_==MAPPING) && (mapping_status_ != STARTPOINT_SET)){
//         ROS_INFO("skip, please make sure: slam_status == STARTPOINT_SET !");
//         return;
//     }else{
//         ROS_INFO("skip, slam status error!");
//         ROS_INFO("running_slam: %u", running_slam_);
//         ROS_INFO("slam_mode: %s", print_SlamMode(slam_mode_).c_str());
//         ROS_INFO("mapping_status: %s", print_MappingStatus(mapping_status_).c_str());
//         ROS_INFO("****************************");
//     }
// }

// void LocalizationModule::stop_mapping(){
//     /////////////////////////////////////////////////////
//     if(running_slam_ && slam_mode_==MAPPING && 
//         (mapping_status_ == ENDPOINT_SET || mapping_status_ == MAPPING_STARTED)){
//         //////////////////////// TODO， 添加保存判断，是否没有地图元素就不保存 GlobalMap.pcd
//         // save global map  (save all to one single map file)
//         ROS_INFO("saving global map");
//         std::string pcd_path = curr_dir_ + std::string("/map/");
//         slam_->save_map(pcd_path, 0.1, 0, 0);
//         ROS_INFO("start stop mapping");
//         // control_status_.reset = true;
//         running_slam_ = false;
//         mapping_status_ = M_INACTIVE;
//         sleep(1);
//         release_slam_obj();
//         ROS_INFO("mapping stopped !");
//     }else if (running_slam_ && slam_mode_==MAPPING && mapping_status_ == STARTPOINT_SET){
//         ROS_INFO("mapping_status: %s", print_MappingStatus(mapping_status_).c_str());
//         ROS_INFO("skip, please finish current map-element, or delete it first !");
//     }else if (!running_slam_){
//         cout << "skip, not running slam !"<<endl;
//         return;
//     }else if(running_slam_ && slam_mode_==LOCALIZATION){
//         ROS_INFO("skip, running slam (mode: localization)!");
//         return;
//     }else {
//         ROS_INFO("skip, slam status error!");
//         ROS_INFO("running_slam: %u", running_slam_);
//         ROS_INFO("slam_mode: %s", print_SlamMode(slam_mode_).c_str());
//         ROS_INFO("mapping_status: %s", print_MappingStatus(mapping_status_).c_str());
//         ROS_INFO("****************************");
//     }
//     /////////////////////////-used when testing-////////////////////////////
//     // int save_id = 1;
//     // std::string pcd_path = curr_dir_ + std::string("/map/") + std::to_string(save_id)+std::string("/");
//     // // 判断 slam_ 
//     // if (save_id > 0){
//     //     slam_->save_map(pcd_path, 0.1, 0, 0);
//     // }
//     /////////////////////////////////////////////////////
//     // //////////////////-debug-////////////////////////
//     // const std::string work_path = curr_dir_+std::string("/");
//     // // slam_->reset(work_path,localization_mode_,offline_mode_);
//     // bool flag = (slam_==nullptr);
//     // cout<<"if slam_==nullptr: "<< flag <<endl;
//     // lidar_slam::LidarSlam *temp_slam = slam_.release();
//     // ROS_INFO("release successfully");
//     // flag = (slam_==nullptr);
//     // cout<<"if slam_==nullptr: "<< flag <<endl;
//     // delete temp_slam;
//     // temp_slam=nullptr;
//     // ROS_INFO("delete successfully");
//     // flag = (slam_==nullptr);
//     // cout<<"if slam_==nullptr: "<< flag <<endl;
//     ////////////////////////////////////////////////////////////
//     return;
// }


// void LocalizationModule::start_localization(bool localization_mode, int map_id){
//     // 
//     ROS_INFO("trying to create lidar_slam, localization_mode: %u", localization_mode);
//     std::string workpath = curr_dir_+std::string("/");
//     std::string pcd_path = curr_dir_+std::string("/map/")+std::to_string(map_id)+std::string("/");
//     if(!running_slam_){
//         // slam
//         bool second_mapping=false;
//         make_slam_obj(workpath, localization_mode, offline_mode_, second_mapping);
//         // 加载地图
//         ROS_INFO("load map dir: %s", pcd_path.c_str());
//         slam_ -> load_map(pcd_path);
//         running_slam_ = true;
//         localization_mode_ = 1;
//     }else if(running_slam_ && slam_mode_ == MAPPING){	
//         ROS_INFO("skip, running mapping now, please stop mapping first!");
//     }else{
//         ROS_INFO("skip, slam status error!");
//         ROS_INFO("running_slam: %u", running_slam_);
//         ROS_INFO("slam_mode: %s", print_SlamMode(slam_mode_).c_str());
//         ROS_INFO("mapping_status: %s", print_MappingStatus(mapping_status_).c_str());
//         ROS_INFO("****************************");
//     }
// }


// void LocalizationModule::stop_localization(){
//     if(running_slam_ && slam_mode_==LOCALIZATION){
//         running_slam_ = false;
//         // control_status_.reset = true;
//         sleep(1);
//         release_slam_obj();
//         ROS_INFO("localization stopped !");
//     }else if (!running_slam_){
//         ROS_INFO("skip, not running slam, return !");
//         return;
//     }else if(running_slam_ && slam_mode_==MAPPING){
//         ROS_INFO("skip, running slam (mode: mapping), return !");
//         return;
//     }else {
//         ROS_INFO("skip, slam status error!");
//         ROS_INFO("running_slam: %u", running_slam_);
//         ROS_INFO("slam_mode: %s", print_SlamMode(slam_mode_).c_str());
//         ROS_INFO("****************************");
//     }
// }

}// namespace localization_module
