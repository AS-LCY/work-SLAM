
#include "node/localization_module.h"


namespace localization_module {
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
     *     START_RELOCALIZATION    = 9000,  // 重定位，定位过程中，重新进行重定位
     *     CMD_MAX
     * };
     *      
     * to be continued
     */  
    //// topic-name: "/flbot/localization_module/ctrl_cmd"

    auto msg = msg_in;
    int ctrl_type = msg.data/1000 * 1000;
    auto curr_cmd = static_cast<SlamCtrlCmd>(ctrl_type);
    ROS_INFO("\033[1;32mReceived Ctrl Cmd: %s \033[0m", print_SlamCtrlCmd(curr_cmd).c_str());

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
                // cout<<"debug: running_module_status_ set"<<endl;
            }else{
                running_module_status_ = last_running_module_status_;
                ROS_ERROR("Start Sec-mapping failed!");
                release_slam_obj();
                ROS_WARN("slam obj destroyed!");
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
                running_module_status_ = last_running_module_status_;
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
                running_module_status_ = last_running_module_status_;
            }

            last_running_module_status_ = MODULE_IDLE;
            break;
        }
        case START_RELOCALIZATION:{// 重新进行重定位
            if(start_relocalization()){
                
            }

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
    // use_ele_pcd_flag: set to false; true: not support for now
    if (use_ele_pcd_flag){
        load_map_dir = slam_param_.common.map_directory +std::string("/")+std::to_string(map_id)+std::string("/");
    }else{
        load_map_dir = slam_param_.common.map_directory +std::string("/");
    }


    if (last_running_module_status_ == MODULE_IDLE){
        make_slam_obj(slam_param_, set_status);
        // 加载地图
        // ROS_INFO("load map dir: %s", load_map_dir.c_str());// 这种方式打印中文字符会乱码，显示为一堆问号，std::cout 可以正常打印中文
        if(!slam_ -> load_map(load_map_dir)){
            ROS_ERROR("load map failed!");
            return false;
        }
        // mapping_status_ = M_STANDBY;
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
            ROS_INFO("\033[1;32mstart saving map data\033[0m");
            std::string pcd_dir = slam_param_.common.map_directory;
            const auto resolution = slam_param_.mapping.save_map_resolution;
            if(!slam_->save_map(pcd_dir, resolution, 0, 0)){
                ROS_ERROR("save map data failed!");
            }else{
                ROS_INFO("\033[1;32msave map data success!\033[0m");
            }

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
        // ROS_INFO("load map dir: %s", load_map_dir.c_str());
        // slam_ -> load_map(load_map_dir);
        if(!slam_ -> load_map(load_map_dir)){
            ROS_ERROR("load map failed!");
            return false;
        }
        show_load_map_=0;
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


bool LocalizationModule::start_relocalization(){
    if(running_module_status_ == MODULE_LOCALIZATION){
        bool glo_success_flag = false;
        slam_->reset_globalLocalizationSuccess(glo_success_flag);
        ROS_INFO("globalLocalizationSuccess reset");
        return true;
    }else if(running_module_status_==MODULE_IDLE || 
             running_module_status_==MODULE_MAPPING){
        ROS_INFO("skip, can not stop localization, running_module_status_: %s", print_ModuleStatus(running_module_status_).c_str());
        return false;
    }else{
        ROS_INFO("skip, status error!");
        ROS_INFO("running_module_status_: %s", print_ModuleStatus(running_module_status_).c_str());
        ROS_INFO("mapping_status_: %s", print_MappingStatus(mapping_status_).c_str());
        ROS_INFO("localization_status_: %s", print_LocalizationStatus(localization_status_).c_str());
        ROS_INFO("****************************");
        return false;
    }
    
}

bool LocalizationModule::make_slam_obj(lidar_slam::LidarSlamParam yaml_param, ModuleStatus set_status){
    // ROS_INFO("debug: making obj: lidar_slam ");
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
    ROS_INFO("Making obj(lidar_slam) --- with: set_slam_mode = %s", lidar_slam::print_SlamWorkMode(set_slam_mode).c_str());
    slam_ = std::make_unique<lidar_slam::LidarSlam>(yaml_param, set_slam_mode);
    ROS_INFO("\033[1;32mMake obj(lidar_slam) successfully !\033[0m");
    return true;
}

void LocalizationModule::release_slam_obj(){
    if (set_module_status_ != MODULE_IDLE){
        ROS_INFO("skip, set module status(== %s) must be MODULE_IDLE before release slam obj", print_ModuleStatus(set_module_status_).c_str());
        return;
    }
    sleep(1);
    ROS_INFO("start stopping lidar_slam");

    // std::unique_ptr<lidar_slam::LidarSlam> temp_slam;
    // temp_slam = std::move(slam_);
    // ROS_INFO("move successfully");

    // slam_.reset();


    lidar_slam::LidarSlam *temp_slam = slam_.release();
    // cout<<"debug: temp_slam: "<<temp_slam<<endl;
    // ROS_INFO("debug: release successfully");
    delete temp_slam;
    cout<<"temp_slam: "<<temp_slam<<endl;
    // ROS_INFO("debug: delete successfully");
    temp_slam = nullptr;
    // ROS_INFO("debug: set nullptr successfully");

    start_index_ = -1;
    end_index_ = -1;
    running_module_status_ = MODULE_IDLE;
    mapping_status_ = M_INACTIVE;
    localization_status_ = L_INACTIVE;
    ROS_INFO("\033[1;32mlidar_slam stopped !\033[0m");
}
}// namespace localization_module