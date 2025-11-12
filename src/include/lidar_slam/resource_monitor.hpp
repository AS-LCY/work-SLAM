#include <logTracer/tracer.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

class ProcessMonitor {
	DECL_CLASSNAME(ProcessMonitor)
   public:
	ProcessMonitor(double interval_sec = 2.0) : running_(true), interval_sec_(interval_sec) {
		monitor_thread_ = std::thread(&ProcessMonitor::monitorLoop, this);
	}

	~ProcessMonitor() {
		running_ = false;
		if (monitor_thread_.joinable()) {
			monitor_thread_.join();
		}
	}

   private:
	struct ProcStat {
		unsigned long utime = 0;
		unsigned long stime = 0;
		unsigned long cutime = 0;
		unsigned long cstime = 0;
	};

	ProcStat readProcStat() {
		std::ifstream file("/proc/self/stat");
		ProcStat stat;
		std::string tmp;
		for (int i = 0; i < 13; ++i) file >> tmp; // skip first 13 fields
		file >> stat.utime >> stat.stime >> stat.cutime >> stat.cstime;
		return stat;
	}

	double getProcessCPUUsage() {
		ProcStat s1 = readProcStat();
		long ticks_per_sec = sysconf(_SC_CLK_TCK);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		ProcStat s2 = readProcStat();

		double total_time = (s2.utime + s2.stime) - (s1.utime + s1.stime);
		double cpu_usage = (total_time / (ticks_per_sec * 0.2)) * 100.0 / sysconf(_SC_NPROCESSORS_ONLN);
		return cpu_usage;
	}

	double getProcessMemoryUsageMB() {
		std::ifstream file("/proc/self/status");
		std::string key;
		unsigned long rss = 0;
		while (file >> key) {
			if (key == "VmRSS:") {
				file >> rss;
				break;
			} else {
				file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			}
		}
		return rss / 1024.0; // KB -> MB
	}

	void monitorLoop() {
		while (running_) {
			double cpu = getProcessCPUUsage();
			double mem = getProcessMemoryUsageMB();
			TRACE_INFO_CLASS("[ProcessMonitor] CPU: %f %, Memory: %f MB", cpu, mem);
			std::this_thread::sleep_for(std::chrono::duration<double>(interval_sec_));
		}
	}

   private:
	std::atomic<bool> running_;
	std::thread monitor_thread_;
	double interval_sec_;
};