
#include <sys/resource.h>

#include <csignal>
#include <rclcpp/rclcpp.hpp>

#include "node/localization_module.h"
#define SAVE_CORE_DUMP

#define CORE_SIZE 1024 * 1024 * 500 * 1.2

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rcutils_logging_set_logger_level("rclcpp", RCUTILS_LOG_SEVERITY_DEBUG);
	// 创建节点时添加选项
	rclcpp::NodeOptions options;
	options.automatically_declare_parameters_from_overrides(false);
	auto node = std::make_shared<rclcpp::Node>("localization_module", options);

#ifdef SAVE_CORE_DUMP
	RCLCPP_INFO(node->get_logger(), "save core dump is enable");
	// 程序崩溃核心转储
	struct rlimit rlmt;
	if (getrlimit(RLIMIT_CORE, &rlmt) == -1) {
		return -1;
	}

	RCLCPP_INFO(node->get_logger(), "Before set rlimit CORE dump current is:%d, max is:%d", (int)rlmt.rlim_cur,
				(int)rlmt.rlim_max);

	rlmt.rlim_cur = (rlim_t)CORE_SIZE;
	rlmt.rlim_max = (rlim_t)CORE_SIZE;
	if (setrlimit(RLIMIT_CORE, &rlmt) == -1) {
		return -1;
	}
	if (getrlimit(RLIMIT_CORE, &rlmt) == -1) {
		return -1;
	}

	RCLCPP_INFO(node->get_logger(), "After set rlimit CORE dump current is:%d, max is:%d", (int)rlmt.rlim_cur,
				(int)rlmt.rlim_max);
#endif

	RCLCPP_INFO(node->get_logger(), "\033[1;32m----> ros init \033[0m");

	// 设置locale为默认值，以支持当前系统的默认编码
	setlocale(LC_ALL, "");

	int init_module_status = 0;
	node->declare_parameter<int>("common.init_module_status", 0);
	node->get_parameter("common.init_module_status", init_module_status);
	localization_module::ModuleStatus init_status = static_cast<localization_module::ModuleStatus>(init_module_status);

	RCLCPP_INFO(node->get_logger(), "\033[1;32m----> localization_module starting! \033[0m");
	auto localization_node = std::make_shared<localization_module::LocalizationModule>(node, init_status);

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	RCLCPP_INFO(rclcpp::get_logger("thread_number"), "number of threads: %ld", executor.get_number_of_threads());
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
