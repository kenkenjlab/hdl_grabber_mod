#ifndef HDL_CLUSTERER_VERSION
#define HDL_CLUSTERER_VERSION 2014091001


// ------------------------------------------
//		Includes
// ------------------------------------------

#include "hdl_manager.hpp"
#include <pcl/search/kdtree.h>
#include <omp.h>


// ------------------------------------------
//		Classes
// ------------------------------------------

template<class PointType>
class HDLClusterer : public HDLManager<PointType> {
private:
	typedef pcl::PointXYZRGBA Index;	// Use rgba field as index (uint32_t)
	typedef pcl::PointCloud<Index> Indices;
	typedef typename Indices::Ptr IndicesPtr;
	typedef typename Indices::ConstPtr IndicesConstPtr;
	typedef pcl::search::KdTree<PointType> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;
	
	//// Protected Fields ////
	boost::mutex indices_mutex_;
	boost::signals2::connection indices_connection_;
	IndicesConstPtr indices_;

	//// Protected Methods ////
	void indicesCallback_ (const IndicesConstPtr& indices) {
		boost::mutex::scoped_lock lock (indices_mutex_);
		indices_ = indices;
	}

	template <class ScalarT> std::vector<ScalarT> differentiate_(const std::vector<ScalarT> &values);

public:
	//// Public Methods ////
	HDLClusterer() : HDLManager<PointType>() {}
	~HDLClusterer() {}

	// Setter
	int open(const std::string &uri = "", const std::string &calib_file = "");

	// Processing
	int getPointCloud(CloudConstPtr &cloud, pcl::PointCloud<pcl::PointXYZRGBA> &special_cloud);
};


// ------------------------------------------
//		Methods
// ------------------------------------------

template<class PointType>
int HDLClusterer<PointType>::open(const std::string &uri, const std::string &calib_file) {
	// (1) Open
	int ret = HDLManager<PointType>::open(uri, calib_file);
	
	// (2) Set laser indices as colors
	pcl::RGB color;
	unsigned int &index = color.rgba;
	index = 0;
	for(int i = 0; i < getLasersNum(); i += 2, index++)
		grabber_->setLaserColorRGB(color, i);
	for(int i = 1; i < getLasersNum(); i += 2, index++)
		grabber_->setLaserColorRGB(color, i);

	// (3) Set a callback method
	boost::function<void (const IndicesConstPtr&)> indices_cb = boost::bind(&HDLClusterer<PointType>::indicesCallback_, this, _1);
	indices_connection_ = grabber_->registerCallback(indices_cb);
	
	return ret;
}

template<class PointType>
int HDLClusterer<PointType>::getPointCloud(CloudConstPtr &cloud, pcl::PointCloud<pcl::PointXYZRGBA> &special_cloud) {
	// (1) Get raw point cloud
	int ret = HDLManager<PointType>::getPointCloud(cloud);
	if(ret)
		return ret;

	// (2) Get indices cloud
	IndicesConstPtr indices;
	if(!indices_mutex_.try_lock()) {
		if(isDebugMode())
			std::cerr << "ERROR: Failed to get indices cloud" << std::endl;
		return 1;
	}
	indices = indices_;
	IndicesConstPtr new_indices;
	indices_.swap(new_indices);
	indices_mutex_.unlock();

	// (3) Cluster
	special_cloud.clear();
	if(cloud && indices) {
		// (a) Create indices table from cloud (which laser the points belongs to)
		std::vector<std::vector<int>> indices_table(getLasersNum());
		for(int i = 0; i < indices->size(); i++) {
			int index = indices->points[i].rgba;
			indices_table[index].push_back(i);
		}

		// (b) For each laser
		for(int i = 0; i < indices_table.size(); i++) {
			CloudPtr cloud_on_laser(new Cloud);
			pcl::copyPointCloud(*cloud, indices_table[i], *cloud_on_laser);
			
			// (A) Compute distances
			std::vector<double> distances, angles;
			BOOST_FOREACH(const PointType &p, *cloud_on_laser) {
				Eigen::Vector3f v = p.getVector3fMap();
				distances.push_back(v.norm());
				angles.push_back(std::atan2(v(1), v(0)));
			}
			int size = distances.size();

			// (B) Compute from 1st to 3rd order differentiation of distances
			std::vector<double> distances_diff_1, distances_diff_2, distances_diff_3;
			distances_diff_1 = differentiate_(distances);
			distances_diff_2 = differentiate_(distances_diff_1);
			distances_diff_3 = differentiate_(distances_diff_2);

			// (C) Find zero crossing points
			for(int i = 0; i < size; i++) {
				int prev_i = (i == 0) ? size : i - 1;
				//if(distances_diff_3[i] * distances_diff_3[prev_i] <= 0) {
				if(std::fabs(distances_diff_1[i]) > 0.1) {
				/*
				const PointType &p1 = cloud_on_laser->points[prev_i];
				const PointType &p2 = cloud_on_laser->points[i];
				Eigen::Vector3f v1 = p1.getVector3fMap();
				Eigen::Vector3f v2 = p2.getVector3fMap();
				if((v1 - v2).norm() > 0/1) {
				*/
					pcl::PointXYZRGBA p_c;
					pcl::copyPoint(cloud_on_laser->points[i], p_c);
					p_c.r = 255;
					p_c.g = p_c.b = 0;
					special_cloud.push_back(p_c);
				}
			}
		}

	}
	return 0;
}

template <class PointType>
template <class ScalarT>
std::vector<ScalarT> HDLClusterer<PointType>::differentiate_(const std::vector<ScalarT> &values) {
	int size = values.size();
	std::vector<ScalarT> diffs(size);
	for(int i = 0; i < size; i++) {
		int prev_i = (i == 0) ? size : i - 1;
		diffs[i] = values[i] - values[prev_i];
	}
	return diffs;
}

#endif
