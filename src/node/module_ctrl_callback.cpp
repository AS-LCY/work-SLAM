
#include "node/localization_module.h"
#include "localization_module.h"


namespace localization_module {
void LocalizationModule::localization_module_ctrl_callback(const std_msgs::UInt32 &msg_in){
    if(slam_param_.common.cpu_id.size()>0){
        pthread_t this_thread = pthread_self(); // 获取当前线程的 ID
        if (pthread_setaffinity_np(this_thread, sizeof(cpu_mask_), &cpu_mask_) < 0) {
            perror("pthread_setaffinity_np");
            exit(EXIT_FAILURE);
        }
    }
    hb_time_cbk_module_ctrl_.store(ros::Time::now().toSec());

    /** msg_in *************************************************************************************
     * enum SlamCtrlCmd{
     *     START_MAPPING           = 1000,  // 开始建图
     *     START_SEC_MAPPING       = 2000,  // 重定位->建图，二次建图
     *     EXIT_MAPPING            = 3000,  // 退出建图
     *     START_LOCALIZATION      = 7000,  // localization, 重定位->定位
     *     EXIT_LOCALIZATION       = 8000,  // exit localization, 退出定位
     *     START_RELOCALIZATION    = 9000,  // relocalization, 重定位，定位过程中，重新进行重定位
     *     RESTART_SEC_MAPPING     = 9100,  // restart sec-mapping, 重启二次建图（一般是二次建图重定位失败的情况）
     *     CMD_MAX
     *     [MAPPING_POINT_BEGIN]     = 3000,  // invalid, 设置起点
     *     [MAPPING_ELE_DELETE ]     = 4000,  // invalid, 创建地图元素过程中，清除当前元素（当前元素还未完成创建）
     *     [MAPPING_POINT_END  ]     = 5000,  // invalid, 设置终点，带子地图ID，5001，ID=1
     * };
     *      
     * to be continued
     */  
    //// topic-name: "/flbot/localization_module/ctrl_cmd" *****************************************

    auto msg = msg_in;
    int ctrl_type = msg.data/100 * 100;
    auto curr_cmd = static_cast<SlamCtrlCmd>(ctrl_type);
    ROS_INFO_STREAM(BOLDGREEN << "Received Ctrl Msg: " << msg << RESET;);
    ROS_INFO_STREAM(BOLDGREEN << "Received Ctrl Cmd: " << print_SlamCtrlCmd(curr_cmd) << RESET);
    // ROS_INFO_STREAM(" CMD_MAX: " << CMD_MAX);

    int map_id = msg.data % 100;
    if(map_id == 0) {map_id=1;}

    

    switch (curr_cmd){
        case START_MAPPING:{// 初次建图，或重置后建图
            if(start_mapping(map_id)){//启动建图成功
                ROS_INFO_STREAM(GREEN << "start_mapping success!" <<RESET);
            }else{//启动建图失败
                ROS_ERROR_STREAM(RED << "start_mapping failed!" <<RESET);
            }

            break;
        }
        case START_SEC_MAPPING:{// 重定位，并开始建图，/// TODO/////////////////////////////////////
            if(start_second_mapping( map_id)){
                ROS_INFO_STREAM(GREEN << "start_second_mapping success!" <<RESET);
            }else{             
                ROS_ERROR_STREAM(RED << "Start Sec-mapping failed!" <<RESET);
                release_slam_obj();
                ROS_WARN_STREAM(YELLOW << "slam obj destroyed!"<< RESET);
            }
            break;
        }
        case EXIT_MAPPING:{// 退出建图           
            if(stop_mapping()){
                ROS_INFO_STREAM(GREEN << "stop_mapping success!" <<RESET);
            }else{
                ROS_ERROR_STREAM(RED << "stop_mapping failed!" <<RESET);
            }
            break;
        }
        case START_LOCALIZATION:{// 开始定位，（先重定位，再定位）          
            if(start_localization(map_id)){
                ROS_INFO_STREAM(GREEN << "start_localization success!" <<RESET);
            }else{
                ROS_ERROR_STREAM(RED << "start_localization failed!" <<RESET);
            }
            break;
        }
        case EXIT_LOCALIZATION:{// 退出定位          
            if(stop_localization()){// 停止定位成功
                ROS_INFO_STREAM(GREEN << "stop_localization success!" <<RESET);
            }else{
                ROS_ERROR_STREAM(RED << "stop_localization failed!" <<RESET);
            }
            break;
        }
        case START_RELOCALIZATION:{// 重新进行重定位
            if(start_relocalization(map_id)){
                ROS_INFO("restart localization successfully!");
            }else{
                ROS_ERROR_STREAM(RED << "start_relocalization failed!" <<RESET);
            }

            break;
        }
        case RESTART_SEC_MAPPING:{// 重新进行重定位
            // int map_id = msg.data % 100;
            if(restart_second_mapping(map_id)){
                ROS_INFO("restart sec_mapping successfully!");
            }else{
                ROS_ERROR_STREAM(RED << "restart sec_mapping failed!" <<RESET);
            }
            break;
        }

        default:{
            ROS_INFO("mapping ctrl msg: %u invalid!", msg.data);
            break;
        }
    }
}

// bool LocalizationModule::start_mapping(ModuleStatus set_status){
bool LocalizationModule::start_mapping(int map_id){
    auto set_status = ModuleStatus::MODULE_MAPPING;
    auto running_module_status_now = running_module_status_.load();
    ROS_INFO("Module Status for now: %s", print_ModuleStatus(running_module_status_now).c_str());
    ROS_INFO("Trying to set module_status: %s", print_ModuleStatus(set_status).c_str());

    if(!make_map_directory_name(map_id)){// 基于 map_id, 保存在 [slam_param_.common.map_directory]
        ROS_ERROR_STREAM("Map Directory Error!");
        exit(EXIT_FAILURE); 
    }
    if (running_module_status_now == ModuleStatus::MODULE_IDLE){
        running_module_status_.store(ModuleStatus::MODULE_STARTING_SLAM);
        make_slam_obj(slam_param_, set_status);
        // mapping_status_ = M_STANDBY;
        // running_module_status_ = ModuleStatus::MODULE_MAPPING;   
        running_module_status_.store(ModuleStatus::MODULE_MAPPING);
        mapping_node_status_.store(1);// 1: normal
        return true;
    }else if(is_mapping_status(running_module_status_now)){
        ROS_INFO("skip, already running mapping now");//////////////////// TODO 要不要重置 start_index_ end_index_ ？？？
        return false;
    }else if (running_module_status_now == ModuleStatus::MODULE_LOCALIZATION){
        ROS_INFO("skip, running localizing now, please stop localizing first");
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("running_module_status_now: %s", print_ModuleStatus(running_module_status_now).c_str());
        ROS_INFO("set_module_status: %s", print_ModuleStatus(set_status).c_str());
        ROS_INFO("mapping_status: %d", mapping_status_.load());
        ROS_INFO("****************************");
        return false;
    }

}


bool LocalizationModule::start_second_mapping(int map_id){
    auto running_module_status_now = running_module_status_.load();
    ModuleStatus set_status = ModuleStatus::MODULE_SEC_MAPPING;

    ROS_INFO("Module Status for now: %s", print_ModuleStatus(running_module_status_now).c_str());
    ROS_INFO("Trying to set module_status: %s", print_ModuleStatus(set_status).c_str());

    if(!make_map_directory_name(map_id)){// 基于 map_id, 保存在 [slam_param_.common.map_directory]
        ROS_ERROR_STREAM("Map Directory Error!");
        exit(EXIT_FAILURE);
    }
    std::string load_map_dir = slam_param_.common.cloud_map_directory;

    if (running_module_status_now == ModuleStatus::MODULE_IDLE){
        running_module_status_.store(ModuleStatus::MODULE_STARTING_SLAM);
        make_slam_obj(slam_param_, set_status);
        // 加载地图
        // ROS_INFO("load map dir: %s", load_map_dir.c_str());// 这种方式打印中文字符会乱码，显示为一堆问号，std::cout 可以正常打印中文
        if(!slam_ -> load_map(load_map_dir)){
            ROS_ERROR_STREAM(RED << "load map failed!" <<RESET);
            
            usleep(500000); // 单位: 微秒, 500,000us = 500ms = 0.5s
            release_slam_obj();
            ROS_INFO("slam destoried!");

            // update module-status
            running_module_status_.store(ModuleStatus::MODULE_IDLE);

            return false;
        }else{// start_second_mapping success
            running_module_status_.store(ModuleStatus::MODULE_SEC_MAPPING);
            mapping_node_status_.store(1);// 1: normal
            return true;
        }
        // mapping_status_ = M_STANDBY;
        // running_module_status_ = ModuleStatus::MODULE_SEC_MAPPING;
        // return true;
    }else if(is_mapping_status(running_module_status_now)){
        ROS_INFO("skip, already running mapping now");
        return false;
    }else if (running_module_status_now == ModuleStatus::MODULE_LOCALIZATION){
        ROS_INFO("skip, running localizing now, please stop localizing first");
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("running_module_status_now: %s", print_ModuleStatus(running_module_status_now).c_str());
        ROS_INFO("set_module_status_: %s", print_ModuleStatus(set_status).c_str());
        ROS_INFO("mapping_status: %d", mapping_status_.load());
        ROS_INFO("****************************");
        return false;
    }
}

bool LocalizationModule::stop_mapping_without_saving_map(){
    auto running_module_status_now = running_module_status_.load();
    auto set_status = ModuleStatus::MODULE_IDLE;
    if (is_mapping_status(running_module_status_now)){
        running_module_status_.store(ModuleStatus::MODULE_STOPPING_SLAM);
        // sleep(1); // 暂停一秒
        usleep(500000); // 单位: 微秒, 500,000us = 500ms = 0.5s
        release_slam_obj();
        ROS_INFO("mapping stopped without saving map!");

        running_module_status_.store(ModuleStatus::MODULE_IDLE);
        mapping_node_status_.store(0);// 0: inactive

        return true;

    // }else if(last_running_module_status_ == ModuleStatus::MODULE_IDLE || 
    //          last_running_module_status_ == ModuleStatus::MODULE_LOCALIZATION){
    }else if(running_module_status_now == ModuleStatus::MODULE_IDLE || 
             running_module_status_now == ModuleStatus::MODULE_LOCALIZATION){
        ROS_INFO("skip, can not stop mapping, running_module_status_: %s", print_ModuleStatus(running_module_status_now).c_str());
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("running_module_status_now: %s", print_ModuleStatus(running_module_status_now).c_str());
        ROS_INFO("set_module_status: %s", print_ModuleStatus(set_status).c_str());
        ROS_INFO("mapping_status: %d", mapping_status_.load());
        ROS_INFO("localization_status: %d", localization_status_.load());
        ROS_INFO("****************************");
        return false;
    }
}


bool LocalizationModule::stop_mapping(){
    // TODO: 问题：当点云量为0时，程序挂掉
    // if (last_running_module_status_ == ModuleStatus::MODULE_MAPPING || last_running_module_status_ == ModuleStatus::MODULE_SEC_MAPPING){
    auto running_module_status_now = running_module_status_.load();
    auto mapping_status_now = mapping_status_.load();
    ModuleStatus set_status = ModuleStatus::MODULE_IDLE;
    // if (is_mapping_status(last_running_module_status_)){
    if (is_mapping_status(running_module_status_now)){
        // if(mapping_status_ == M_STANDBY){
        if(mapping_status_now == 3){
            running_module_status_.store(ModuleStatus::MODULE_STOPPING_SLAM);
            ROS_INFO("mapping_status: %d", mapping_status_.load());
            ROS_INFO("\033[1;32mstart saving map data\033[0m");
            std::string pcd_dir = slam_param_.common.cloud_map_directory;
            const auto resolution = slam_param_.mapping.save_map_resolution;
            if(!slam_->save_map(pcd_dir, resolution, 0, 0)){
                ROS_ERROR_STREAM(RED << "save map data failed!" <<RESET);
            }else{
                ROS_INFO("\033[1;32msave map data success!\033[0m");
            }

            map_saved_.store(1);

            ROS_INFO("start stop mapping");
            // mapping_status_ = M_INACTIVE;
            sleep(1);
            // set_module_status_ = ModuleStatus::MODULE_IDLE;
            release_slam_obj();
            ROS_INFO("mapping stopped !");
            running_module_status_.store(ModuleStatus::MODULE_IDLE);
            mapping_node_status_.store(0);// 0: inactive

            return true;
        }else if(mapping_status_now == M_CREATING_ELE){
            ROS_INFO("mapping_status: %d", mapping_status_.load());
            ROS_INFO("skip, please finish current map-element, or delete it first !");
            return false;
        }else{
            ROS_INFO("skip, status error!");
            ROS_INFO("set_module_status: %s", print_ModuleStatus(set_status).c_str());
            ROS_INFO("running_module_status_now: %s", print_ModuleStatus(running_module_status_now).c_str());
            ROS_INFO("mapping_status: %d", mapping_status_.load());
            ROS_INFO("localization_status: %d", localization_status_.load());
            ROS_INFO("****************************");
            return false;
        }
    }else if(running_module_status_now == ModuleStatus::MODULE_IDLE || running_module_status_now == ModuleStatus::MODULE_LOCALIZATION){
        ROS_INFO("skip, can not stop mapping, running_module_status_: %s", print_ModuleStatus(running_module_status_.load()).c_str());
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("running_module_status_now: %s", print_ModuleStatus(running_module_status_now).c_str());
        ROS_INFO("set_module_status: %s", print_ModuleStatus(set_status).c_str());
        ROS_INFO("mapping_status: %d", mapping_status_.load());
        ROS_INFO("localization_status: %d", localization_status_.load());
        ROS_INFO("****************************");
        return false;
    }

    // running_module_status_ = ModuleStatus::MODULE_IDLE;

    // return;
}


bool LocalizationModule::start_localization(int map_id){
    ModuleStatus running_module_status_now = running_module_status_.load();
    ModuleStatus set_status = ModuleStatus::MODULE_LOCALIZATION;
    auto localization_status_now = localization_status_.load();
    
    ROS_INFO("Module Status for now: %s", print_ModuleStatus(running_module_status_now).c_str());
    ROS_INFO("Trying to set module_status: %s", print_ModuleStatus(set_status).c_str());

    if(!make_map_directory_name(map_id)){// 基于 map_id, 保存在 [slam_param_.common.map_directory]
        ROS_ERROR_STREAM("Map Directory Error!");
        exit(EXIT_FAILURE);
    }
    std::string load_map_dir = slam_param_.common.cloud_map_directory;

    // ModuleStatus curr_running_module_status = running_module_status_.load();

    // if (running_module_status_now == ModuleStatus::MODULE_IDLE){
    if (need_start_localization(running_module_status_now, localization_status_now)){
        // update status
        running_module_status_.store(ModuleStatus::MODULE_STARTING_SLAM);

        make_slam_obj(slam_param_, set_status);
        // 加载地图
        // ROS_INFO("load map dir: %s", load_map_dir.c_str());
        if(!slam_ -> load_map(load_map_dir)){
            ROS_ERROR_STREAM(RED << "load map failed!" <<RESET);
            release_slam_obj();

            running_module_status_.store(ModuleStatus::MODULE_IDLE);
            local_node_status_.store(0);// 0 = INACTIVE
            return false;
        }else{
            show_load_map_=0;

            // update status
            running_module_status_.store(ModuleStatus::MODULE_LOCALIZATION);
            local_node_status_.store(1);// 1 = NORMAL
            return true;

        }
    }else if (running_module_status_now == ModuleStatus::MODULE_LOCALIZATION && (localization_status_is_ok(localization_status_now))){
        ROS_INFO("skip, already running localization normally now");
        return false;
    }else if(is_mapping_status(running_module_status_now)){
        ROS_INFO("skip, running mapping now, please stop mapping first");
        return false;
    }else{
        ROS_INFO("skip, status error! start localization failed !");
        ROS_INFO("running_module_status_now: %s", print_ModuleStatus(running_module_status_now).c_str());
        ROS_INFO("set_module_status: %s", print_ModuleStatus(set_status).c_str());
        ROS_INFO("running_module_status_now: %s", print_ModuleStatus(running_module_status_now).c_str());
        ROS_INFO("mapping_status: %d", mapping_status_.load());
        ROS_INFO("****************************");
        return false;
    }
}

bool LocalizationModule::stop_localization(){
    auto running_module_status_now = running_module_status_.load();
    auto set_status = ModuleStatus::MODULE_LOCALIZATION;

    if (running_module_status_now == ModuleStatus::MODULE_LOCALIZATION){
        // update module-status
        running_module_status_.store(ModuleStatus::MODULE_STOPPING_SLAM);

        // sleep(1); // 暂停一秒
        usleep(500000); // 单位: 微秒, 500,000us = 500ms = 0.5s
        release_slam_obj();
        ROS_INFO("localization stopped !");

        // update module-status
        running_module_status_.store(ModuleStatus::MODULE_IDLE);
        local_node_status_.store(0);

        return true;

    }else if(running_module_status_now == ModuleStatus::MODULE_IDLE || 
                running_module_status_now == ModuleStatus::MODULE_MAPPING|| 
                running_module_status_now == ModuleStatus::MODULE_SEC_MAPPING){
        ROS_INFO("skip, can not stop localization, running_module_status_: %s", print_ModuleStatus(running_module_status_now).c_str());
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("last_running_module_status_: %s", print_ModuleStatus(running_module_status_now).c_str());
        ROS_INFO("set_module_status: %s", print_ModuleStatus(set_status).c_str());
        ROS_INFO("running_module_status_: %s", print_ModuleStatus(running_module_status_now).c_str());
        ROS_INFO("mapping_status: %d", mapping_status_.load());
        ROS_INFO("localization_status: %d", localization_status_.load());
        ROS_INFO("****************************");
        return false;
    }
}


bool LocalizationModule::start_relocalization(int map_id){
    ModuleStatus running_module_status_now = running_module_status_.load();
    if(running_module_status_now == ModuleStatus::MODULE_LOCALIZATION){
        // last_running_module_status_ = running_module_status_;
        // set_module_status_ = ModuleStatus::MODULE_IDLE;
        // // running_module_status_ = ModuleStatus::MODULE_STOPPING_SLAM;
        // running_module_status_.store(ModuleStatus::MODULE_STOPPING_SLAM);        
        if(!stop_localization()){
            return false;
        }else{
            if(!start_localization(map_id)){
                return false;
            }else{
                return true;
            }
        }
    }else if(running_module_status_now == ModuleStatus::MODULE_IDLE || 
             running_module_status_now == ModuleStatus::MODULE_MAPPING){
        ROS_INFO("skip, can not stop localization, running_module_status_: %s", print_ModuleStatus(running_module_status_now).c_str());
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("running_module_status_: %s", print_ModuleStatus(running_module_status_now).c_str());
        ROS_INFO("mapping_status: %d", mapping_status_.load());
        ROS_INFO("localization_status: %d", localization_status_.load());
        ROS_INFO("****************************");
        return false;
    }
    
}

bool LocalizationModule::restart_second_mapping( int map_id){
    ModuleStatus curr_running_module_status = running_module_status_.load();

    if(curr_running_module_status == ModuleStatus::MODULE_SEC_MAPPING){    
        if(!stop_mapping_without_saving_map()){
            return false;
        }else{
            if(!start_second_mapping(map_id)) {
                return false;
            }else{
                return true;
            }
        }

    }else if(curr_running_module_status == ModuleStatus::MODULE_IDLE || 
             curr_running_module_status == ModuleStatus::MODULE_MAPPING|| 
             curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION){
        ROS_INFO("skip, can not restart sec_mapping, running_module_status_: %s", print_ModuleStatus(curr_running_module_status).c_str());
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("running_module_status_: %s", print_ModuleStatus(curr_running_module_status).c_str());
        ROS_INFO("mapping_status: %d", mapping_status_.load());
        ROS_INFO("localization_status: %d", localization_status_.load());
        ROS_INFO("****************************");
        return false;
    }
    
}

bool LocalizationModule::make_slam_obj(lidar_slam::LidarSlamParam yaml_param, ModuleStatus set_status){
    // ROS_INFO("debug: making obj: lidar_slam ");
    lidar_slam::SlamWorkMode set_slam_mode = lidar_slam::SlamWorkMode::UNKNOWN;
    if(set_status == ModuleStatus::MODULE_MAPPING){
        set_slam_mode = lidar_slam::SlamWorkMode::MAPPING;
    }else if(set_status == ModuleStatus::MODULE_SEC_MAPPING){
        set_slam_mode = lidar_slam::SlamWorkMode::SEC_MAPPING;
    }else if(set_status == ModuleStatus::MODULE_LOCALIZATION){
        set_slam_mode = lidar_slam::SlamWorkMode::LOCALIZATION;
    }else{
        ROS_ERROR_STREAM(RED << "ModuleStatus: " << print_ModuleStatus(set_status).c_str() << ", status error!" <<RESET);
        return false;
    }
    ROS_INFO("Making obj(lidar_slam) --- with: set_slam_mode = %s", lidar_slam::print_SlamWorkMode(set_slam_mode).c_str());
    slam_ = std::make_unique<lidar_slam::LidarSlam>(yaml_param, set_slam_mode);
    ROS_INFO("\033[1;32mMake obj(lidar_slam) successfully !\033[0m");

    // slipping_ptr_->reset();
    return true;
}

void LocalizationModule::release_slam_obj(){
    releasing_slam_flag_ = true;

    // sleep(1); // 暂停一秒
    usleep(500000); // 单位: 微秒, 500,000us = 500ms = 0.5s
    ROS_INFO("start stopping lidar_slam");


    lidar_slam::LidarSlam *temp_slam = slam_.release();
    // cout<<"debug: temp_slam: "<<temp_slam<<endl;
    // ROS_INFO("debug: release successfully");
    delete temp_slam;
    // cout<<"temp_slam: "<<temp_slam<<endl;
    // ROS_INFO("debug: delete successfully");
    temp_slam = nullptr;
    // ROS_INFO("debug: set nullptr successfully");

    start_index_ = -1;
    end_index_ = -1;
    // running_module_status_ = ModuleStatus::MODULE_IDLE;
    running_module_status_.store(ModuleStatus::MODULE_IDLE);  
    mapping_node_status_.store(0);// 0: inactive
    local_node_status_.store(0);// 0: inactive  
    ROS_INFO("\033[1;32mlidar_slam stopped !\033[0m");
    releasing_slam_flag_ = false;

    // slipping_ptr_->reset();
}

bool LocalizationModule::make_map_directory_name(int map_id){
    std::string map_folder = " ";
    if(slam_param_.common.run_on_mower){
        if (!nh_.getParam("/map_manager/map_folder", map_folder))  {
            // 从 rosparam 读取参数 失败，则用yaml中路径作为默认路径
            ROS_ERROR_STREAM("Load param: /map_manager/map_folder failed, using default param, map_folder: " << slam_param_.common.map_directory );
            map_folder = slam_param_.common.map_directory;
        } else {
            // 成功读取 rosparam, 则选用读取的地图路径
            ROS_INFO_STREAM(GREEN << "Load param: /map_manager/map_folder success: " << map_folder);
        }
    }else{
        // if test on personal computer, use map_directory set in yaml file directly
        map_folder = slam_param_.common.map_directory;
    }

    slam_param_.common.cloud_map_directory = map_folder + std::string("/")+std::to_string(map_id)+std::string("/3dmap/");


    // 检查并创建地图路径
    if (create_directory_if_not_exists(slam_param_.common.map_directory )) {
        ROS_INFO_STREAM("Directory created or already exists: " << slam_param_.common.map_directory );
    } else {
        ROS_ERROR_STREAM(RED << "Failed to create directory: " << slam_param_.common.map_directory   <<RESET);
        return false;
    }
    return true;
}

bool LocalizationModule::need_start_localization(ModuleStatus running_module_status_now, int localiztion_status_now){
    if (running_module_status_now == ModuleStatus::MODULE_IDLE){
        return true;
    }else if(running_module_status_now == ModuleStatus::MODULE_LOCALIZATION && localization_status_is_failed(localiztion_status_now)){
        return true;
    }else{
        return false;
    }

}

bool LocalizationModule::localization_status_is_ok(int localiztion_status_now){
    if (localiztion_status_now == 3 || localiztion_status_now == 4){
        return true;
    }else{
        return false;
    }
}
bool LocalizationModule::localization_status_is_failed(int localiztion_status_now){
    if (localiztion_status_now == 2 || localiztion_status_now == 5){
        return true;
    }else{
        return false;
    }
}
bool LocalizationModule::mapping_status_is_ok(int mapping_status_now){
    if (mapping_status_now == 3 ){
        return true;
    }else{
        return false;
    }
}
bool LocalizationModule::mapping_status_is_failed(int mapping_status_now){
    if (mapping_status_now == 2 || mapping_status_now == 5){
        return true;
    }else{
        return false;
    }
}

}// namespace localization_module 