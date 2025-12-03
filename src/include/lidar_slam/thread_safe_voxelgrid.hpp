#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>

#include <Eigen/Core>
#include <cmath>
#include <memory>

namespace lidar_slam {
template <typename PointT>
class ThreadSafeVoxelGrid {
   public:
	using CloudT = pcl::PointCloud<PointT>;
	using CloudPtr = typename CloudT::Ptr;

	ThreadSafeVoxelGrid() : leaf_size_(0.1f, 0.1f, 0.1f) {}

	inline void setLeafSize(float lx, float ly, float lz) { leaf_size_ = Eigen::Vector3f(lx, ly, lz); }

	inline void setInputCloud(const CloudPtr& cloud) { cloud_ = cloud; }

	CloudPtr filter() const {
		CloudPtr output = std::make_shared<CloudT>();
		if (!cloud_ || cloud_->empty()) return output;

		const float lx = leaf_size_.x();
		const float ly = leaf_size_.y();
		const float lz = leaf_size_.z();

		// -----------------------------
		// 聚合数据结构
		// -----------------------------
		struct VoxelData {
			float x = 0, y = 0, z = 0;
			float intensity_sum = 0;
			std::size_t count = 0;

			inline void add(const PointT& p) {
				x += p.x;
				y += p.y;
				z += p.z;
				count++;

				// 自动判断是否含 intensity 字段
				if constexpr (pcl::traits::has_field<PointT, pcl::fields::intensity>::value)
					intensity_sum += p.intensity;
			}
		};

		tbb::concurrent_unordered_map<int64_t, VoxelData> voxels;

		// -----------------------------
		// 哈希函数：高效唯一 key
		// -----------------------------
		auto voxel_hash = [&](float x, float y, float z) -> int64_t {
			int64_t ix = static_cast<int64_t>(std::floor(x / lx));
			int64_t iy = static_cast<int64_t>(std::floor(y / ly));
			int64_t iz = static_cast<int64_t>(std::floor(z / lz));
			return (ix << 42) ^ (iy << 21) ^ iz;
		};

		tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud_->size()), [&](const tbb::blocked_range<size_t>& range) {
			for (size_t i = range.begin(); i != range.end(); ++i) {
				const PointT& p = cloud_->points[i];
				int64_t key = voxel_hash(p.x, p.y, p.z);

				auto res = voxels.insert({ key, VoxelData() });
				res.first->second.add(p);
			}
		});

		output->reserve(voxels.size());

		for (auto& kv : voxels) {
			const auto& v = kv.second;

			PointT pt;
			pt.x = v.x / v.count;
			pt.y = v.y / v.count;
			pt.z = v.z / v.count;

			// 自动判断是否有 intensity 字段
			if constexpr (pcl::traits::has_field<PointT, pcl::fields::intensity>::value)
				pt.intensity = v.intensity_sum / v.count;

			output->push_back(pt);
		}

		output->width = output->size();
		output->height = 1;
		output->is_dense = true;

		return output;
	}

   private:
	CloudPtr cloud_;
	Eigen::Vector3f leaf_size_;
};
} // namespace lidar_slam
