#include "lidar_slam/common_lib.h"

Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R) {
	Eigen::Vector3d n = R.col(0);
	Eigen::Vector3d o = R.col(1);
	Eigen::Vector3d a = R.col(2);

	Eigen::Vector3d ypr(3);
	double y = atan2(n(1), n(0));
	double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
	double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
	ypr(0) = y;
	ypr(1) = p;
	ypr(2) = r;

	return ypr;
}

Eigen::Matrix3d ypr2R(const Eigen::Vector3d& ypr) {
	double y = ypr(0);
	double p = ypr(1);
	double r = ypr(2);

	Eigen::Matrix3d Rz;
	Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;

	Eigen::Matrix3d Ry;
	Ry << cos(p), 0., sin(p), 0., 1., 0., -sin(p), 0., cos(p);

	Eigen::Matrix3d Rx;
	Rx << 1., 0., 0., 0., cos(r), -sin(r), 0., sin(r), cos(r);

	return Rz * Ry * Rx;
}

void get_xyz_ypr(const Eigen::Isometry3d& eigen_transform, Eigen::Vector3d& xyz, Eigen::Vector3d& ypr) {
	double x = eigen_transform.translation().x();
	double y = eigen_transform.translation().y();
	double z = eigen_transform.translation().z();

	xyz[0] = x;
	xyz[1] = y;
	xyz[2] = z;

	ypr = R2ypr(eigen_transform.rotation());

	// yaw   = ypr[0];
	// pitch = ypr[1];
	// roll  = ypr[2];
}

Eigen::Matrix3d rpy2R(const Eigen::Vector3d& rpy) {
	// 初始化欧拉角(Z-Y-X，即RPY, 先绕x轴roll,再绕y轴pitch,最后绕z轴yaw)
	// Eigen::Vector3d eu_ang(roll, pitch, yaw);
	Eigen::Vector3d eu_ang(rpy(0), rpy(1), rpy(2));
	Eigen::AngleAxisd rol_vect(Eigen::AngleAxisd(eu_ang(0), Eigen::Vector3d::UnitX()));
	Eigen::AngleAxisd pit_vect(Eigen::AngleAxisd(eu_ang(1), Eigen::Vector3d::UnitY()));
	Eigen::AngleAxisd yaw_vect(Eigen::AngleAxisd(eu_ang(2), Eigen::Vector3d::UnitZ()));

	Eigen::Matrix3d rot_matrix3d = Eigen::Matrix3d::Identity();
	rot_matrix3d = yaw_vect * pit_vect * rol_vect;

	return rot_matrix3d;
}

Eigen::Matrix3d g2R(const Eigen::Vector3d& g) {
	Eigen::Matrix3d R0;
	Eigen::Vector3d ng1 = g.normalized();
	Eigen::Vector3d ng2{ 0, 0, 1.0 };
	R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
	double yaw = R2ypr(R0).x();
	R0 = ypr2R(Eigen::Vector3d{ -yaw, 0, 0 }) * R0;
	return R0;
}

PointCloudType::Ptr transformPointCloud(PointCloudType::Ptr cloudIn, const Eigen::Isometry3d& transCur) {
	PointCloudType::Ptr cloudOut(new PointCloudType());

	int cloudSize = cloudIn->size();
	cloudOut->resize(cloudSize);

#pragma omp parallel for num_threads(MP_PROC_NUM)
	for (int i = 0; i < cloudSize; ++i) // TODO check eigen faster？
	{
		const auto& pointFrom = cloudIn->points[i];
		cloudOut->points[i].x =
			transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
		cloudOut->points[i].y =
			transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
		cloudOut->points[i].z =
			transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
		cloudOut->points[i].intensity = pointFrom.intensity;
	}
	return cloudOut;
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool mkdir_p(const std::string& path, mode_t mode) {
	// 替换路径中的 "//" 为 "/"
	std::string path_temp = path;
	std::string to_replace = "//";
	std::string replacement = "/";
	std::size_t pos = 0;

	while ((pos = path_temp.find(to_replace, pos)) != std::string::npos) {
		path_temp.replace(pos, to_replace.length(), replacement);
		pos += replacement.length(); // 更新位置，继续查找
	}

	char tmp[256];
	char* p = NULL;
	size_t len;

	// Copy string so we can modify it.
	snprintf(tmp, sizeof(tmp), "%s", path_temp.c_str());
	len = strlen(tmp);

	// Remove trailing slashes.
	// 删除末尾的'/'
	while (len > 1 && tmp[len - 1] == '/') tmp[--len] = 0;

	// Iterate over the path, creating directories as needed.
	// 根据找到的'/'，创建目录
	for (p = tmp + 1; *p; p++) {
		if (*p == '/') {
			*p = 0;
			if (mkdir(tmp, mode) && errno != EEXIST) {
				return false;
			}
			*p = '/';
		}
	}

	// Create the final directory.
	// 由于末尾的'/'已被删除，最低一级目录，循环内不会被创建
	// TODO: 要是不删除最后的'/'，是不是就不用分两步了，待测
	if (mkdir(tmp, mode) && errno != EEXIST) {
		return false;
	}

	return true;
}

/////////////////////////////////////////////////////////////////////////////////
// Used in: Backend & module_ctrl_callback
bool create_directory_if_not_exists(const std::string& directory_path) {
#if 1
	if (0 != access(directory_path.c_str(), 0)) {
		bool status = mkdir_p(directory_path.c_str(), 0777);
		if (status) {
			return true; // 创建目录成功
		} else {
			std::cerr << "Error creating directory: " << directory_path << std::endl;
			return false; // 创建目录失败
		}
	} else {
		// folder exist
		return true;
	}
#endif

#if 0
    std::filesystem::path path(directory_path);

    if (!std::filesystem::exists(path)){
        try {
            std::filesystem::create_directories(path);
            return true; // 创建目录成功
        }catch (const std::filesystem::filesystem_error& ex){
            std::cerr << "Error creating directory: " << ex.what() << std::endl;
            // ROS_ERROR_STREAM(RED << "Error creating directory: " << ex.what()  <<RESET);
            return false; // 创建目录失败
        }
    } else {
        return true; // 目录已存在
    }
#endif
}