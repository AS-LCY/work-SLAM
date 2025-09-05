
#include <logTracer/tracer.h>
#include <sys/resource.h>

#include <chrono>
#include <csignal>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#include "node/localization_module.h"

#define SAVE_CORE_DUMP

#define CORE_SIZE 1024 * 1024 * 500 * 1.2
using namespace tracer_log;

std::string getCurrentDateTime() {
	auto now = std::chrono::system_clock::now();
	std::time_t t = std::chrono::system_clock::to_time_t(now); // 转换为 time_t（系统时间戳）
	std::tm tm{};											   // 转换为本地时间
#ifdef _WIN32
	localtime_s(&tm, &t); // Windows 安全函数
#else
	localtime_r(&t, &tm); // Linux / Unix 安全函数
#endif
	std::ostringstream oss; // 格式化输出
	oss << std::put_time(&tm, "%Y-%m-%d-%H:%M:%S");
	return oss.str();
}

int main(int argc, char** argv) {
	int level = TRACE_LEVEL_INFO;				  // TRACE_LEVEL_DBG,  TRACE_LEVEL_INFO, TRACE_LEVEL_WARN
	string log_sink_name = "localization_module"; // 设置日志输出器名称，一般为日志文件名称
	string log_file_name = "localization_module_" + getCurrentDateTime() + ".log"; // 设置日志保存文件名
	int max_file_byte = 10240000;												   // 设置日志文件大小, 10M
	int max_file_num = 25;														   // 设置最大日志文件个数
	// localization_module.log满后，会自动扩展为localization_module.log.1...n

	Tracer::getInstance()
		.addLogSink(log_sink_name.c_str(), std::make_shared<TracerSink>())
		.setSaveFile(log_file_name.c_str(), max_file_byte, max_file_num);
	Tracer::getInstance().setGlobalLog(log_sink_name.c_str());
	Tracer::getInstance().setLevel(level);

	rclcpp::init(argc, argv);
	rcutils_logging_set_logger_level("rclcpp", RCUTILS_LOG_SEVERITY_DEBUG);

	// 创建节点时添加选项
	rclcpp::NodeOptions options;
	options.automatically_declare_parameters_from_overrides(false);
	auto node = std::make_shared<rclcpp::Node>("localization_module", options);

#ifdef SAVE_CORE_DUMP
	TRACE_INFO("save core dump is enable");

	// 程序崩溃核心转储
	struct rlimit rlmt;
	if (getrlimit(RLIMIT_CORE, &rlmt) == -1) {
		return -1;
	}

	TRACE_INFO("Before set rlimit CORE dump current is:%d, max is:%d", (int)rlmt.rlim_cur, (int)rlmt.rlim_max);

	rlmt.rlim_cur = (rlim_t)CORE_SIZE;
	rlmt.rlim_max = (rlim_t)CORE_SIZE;
	if (setrlimit(RLIMIT_CORE, &rlmt) == -1) {
		return -1;
	}
	if (getrlimit(RLIMIT_CORE, &rlmt) == -1) {
		return -1;
	}
	TRACE_INFO("After set rlimit CORE dump current is:%d, max is:%d", (int)rlmt.rlim_cur, (int)rlmt.rlim_max);

#endif
	TRACE_INFO("----> ros init");

	// 设置locale为默认值，以支持当前系统的默认编码
	setlocale(LC_ALL, "");

	int init_module_status = 0;
	node->declare_parameter<int>("common.init_module_status", 0);
	node->get_parameter("common.init_module_status", init_module_status);
	localization_module::ModuleStatus init_status = static_cast<localization_module::ModuleStatus>(init_module_status);

	TRACE_INFO("----> localization_module starting!");
	auto localization_node = std::make_shared<localization_module::LocalizationModule>(node, init_status);

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	TRACE_INFO("number of threads: %ld", executor.get_number_of_threads());

	executor.spin();
	rclcpp::shutdown();
	return 0;
}
