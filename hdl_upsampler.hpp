#ifndef HDL_UPSAMPLER_VERSION
#define HDL_UPSAMPLER_VERSION 2014081201


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
class HDLUpsampler : public HDLManager<PointType> {
private:
	typedef pcl::PointXYZRGBA Index;	// Use rgba field as index (unsigned int)
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


public:
	//// Public Methods ////
	HDLUpsampler() : HDLManager<PointType>() {}
	~HDLUpsampler() {}

	// Setter
	int open(const std::string &uri = "", const std::string &calib_file = "");

	// Processing
	int getPointCloud(CloudPtr &cloud);
};


// ------------------------------------------
//		Methods
// ------------------------------------------

template<class PointType>
int HDLUpsampler<PointType>::open(const std::string &uri, const std::string &calib_file) {
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
	boost::function<void (const IndicesConstPtr&)> indices_cb = boost::bind(&HDLUpsampler<PointType>::indicesCallback_, this, _1);
	indices_connection_ = grabber_->registerCallback(indices_cb);
	
	return ret;
}

template<class PointType>
int HDLUpsampler<PointType>::getPointCloud(CloudPtr &cloud) {
	// (1) Get raw point cloud
	CloudConstPtr raw_cloud;
	int ret = HDLManager<PointType>::getPointCloud(raw_cloud);
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

	// (3) Upsample
	if(raw_cloud && indices) {
		pcl::copyPointCloud<PointType>(*raw_cloud, *cloud);
		std::vector<std::vector<int>> indices_table(getLasersNum());
		for(int i = 0; i < indices->size(); i++) {
			int index = indices->points[i].rgba;
			indices_table[index].push_back(i);
		}
#pragma omp parallep for
		for(int i = 0; i < indices_table.size() - 1; i++) {
			CloudPtr cloud1(new Cloud), cloud2(new Cloud);
			pcl::copyPointCloud(*raw_cloud, indices_table[i], *cloud1);
			pcl::copyPointCloud(*raw_cloud, indices_table[i+1], *cloud2);
			KdTreePtr tree1(new KdTree);
			tree1->setInputCloud(cloud1);
			for(int j = 0; j < cloud2->size(); j++) {
				std::vector<int> inliers;
				std::vector<float> distances;
				tree1->nearestKSearch(cloud2->points[j], 1, inliers, distances);
				Eigen::Vector3f v1 = cloud1->points[inliers[0]].getVector3fMap(),
					v2 = cloud2->points[j].getVector3fMap(),
					v3 = (v1 + v2) / 2;
				PointType p;
				p.x = v3.x();
				p.y = v3.y();
				p.z = v3.z();
				p.intensity = 255;	// Added points will be blue
#pragma omp critical
				{
					cloud->push_back(p);
				}

			}
		}
	}
	return 0;
}

#endif
