#ifndef HDL_LASER_MANAGER_VERSION
#define HDL_LASER_MANAGER_VERSION 2014091101


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
class HDLLaserManager : public HDLManager<PointType> {
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

public:
	//// Public Methods ////
	HDLLaserManager() : HDLManager<PointType>() {}
	~HDLLaserManager() {}

	// Setter
	int open(const std::string &uri = "", const std::string &calib_file = "");

	// Processing
	int getPointCloud(CloudConstPtr &cloud, std::vector<std::vector<int>> &indices_table);
};


// ------------------------------------------
//		Methods
// ------------------------------------------

template<class PointType>
int HDLLaserManager<PointType>::open(const std::string &uri, const std::string &calib_file) {
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
	boost::function<void (const IndicesConstPtr&)> indices_cb = boost::bind(&HDLLaserManager<PointType>::indicesCallback_, this, _1);
	indices_connection_ = grabber_->registerCallback(indices_cb);
	
	return ret;
}

template<class PointType>
int HDLLaserManager<PointType>::getPointCloud(CloudConstPtr &cloud, std::vector<std::vector<int>> &indices_table) {
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
	if(cloud && indices) {
		// (a) Create indices table from cloud (which laser the points belongs to)
		indices_table.resize(getLasersNum());
		for(int i = 0; i < indices->size(); i++) {
			int index = indices->points[i].rgba;
			indices_table[index].push_back(i);
		}
	}
	return 0;
}

#endif
