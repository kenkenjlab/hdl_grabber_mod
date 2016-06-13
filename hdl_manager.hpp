#ifndef HDL_MANAGER_VERSION
#define HDL_MANAGER_VERSION 2015011301


// ------------------------------------------
//		Includes
// ------------------------------------------

#include <pcl/point_cloud.h>
//#include <pcl/io/hdl_grabber.h>
#include "hdl_grabber_mod.h"
#include <boost/algorithm/string.hpp>


// ------------------------------------------
//		Classes
// ------------------------------------------

template<class PointType>
class HDLManager {
public:
	typedef pcl::PointCloud<PointType> Cloud;
	typedef typename Cloud::Ptr CloudPtr;
	typedef typename Cloud::ConstPtr CloudConstPtr;

protected:
//	pcl::HDLGrabber* grabber_;
	HDLGrabber* grabber_;

private:
	bool debug_mode_;
	int laser_num_;
	CloudConstPtr cloud_;
	boost::mutex cloud_mutex_;
	boost::signals2::connection cloud_connection_;

	void cloudCallback_ (const CloudConstPtr& cloud) {
		boost::mutex::scoped_lock lock (cloud_mutex_);
		cloud_ = cloud;
	}

public:
	HDLManager()
		: debug_mode_(false)
		, grabber_(nullptr)
		, laser_num_(64)
	{}
	~HDLManager() { close(); }

	// Setter
	inline void setDebugMode(bool flag = true) { debug_mode_ = flag; }
	inline void setSyncMode(bool flag = true) { if(grabber_) { grabber_->setSyncMode(flag); } }
	inline void setUpsampleMode(bool flag = true) { if(grabber_) { grabber_->setUpsampleMode(flag); } }
	inline void useAverageIntensityForUpsampling(bool flag = true) { if(grabber_) { grabber_->useAverageIntensityForUpsampling(flag); } }
	inline void setUpsampleMaxSquaredDistanceThreshold(float sq_dist) { if(grabber_) { grabber_->setUpsampleMaxSquaredDistanceThreshold(sq_dist); } }
	inline void setUpsampleNumOfPoints(unsigned int num) { if(grabber_) { grabber_->setUpsampleNumOfPoints(num); } }
	inline void setMinimumDistanceThreshold(float distance) { if(grabber_) { grabber_->setMinimumDistanceThreshold(distance); } }
	inline void setMaximumDistanceThreshold(float distance) { if(grabber_) { grabber_->setMaximumDistanceThreshold(distance); } }
	inline float getMinimumDistanceThreshold() const { return grabber_ ? grabber_->getMinimumDistanceThreshold() : -1.0; }
	inline float getMaximumDistanceThreshold() const { return grabber_ ? grabber_->getMaximumDistanceThreshold() : -1.0; }

	// Processing
	int open(const std::string &uri = "", const std::string &calib_file = "", bool sync = false, bool upsample = false, bool average_intensity = false);
	int close();
	int start();
	int stop();
	inline void acquire() { if(grabber_) { grabber_->acquire(); } }
	int getPointCloud(CloudConstPtr &cloud);

	// Getter
	inline bool isDebugMode() const { return debug_mode_; }
	inline bool isOpened() const { return grabber_ != nullptr; }
	inline bool isRunning() const { return isOpened() ? grabber_->isRunning() : false; }
	inline int getLasersNum() const { return laser_num_; }
	inline bool isSyncMode() const { return grabber_ ? grabber_->isSyncMode() : false; }
	inline bool isUpsampleMode() const { return grabber_ ? grabber_->isUpsampleMode() : false; }
	inline bool isUsingAverageIntensityForUpsampling() const { return grabber_ ? grabber_->isUsingAverageIntensityForUpsampling() : false; }
	inline float getUpsampleMaxSquaredDistanceThreshold() const { return grabber_ ? grabber_->getUpsampleMaxSquaredDistanceThreshold() : -1.0; }
};


// ------------------------------------------
//		Public Methods
// ------------------------------------------

template<class PointType>
int HDLManager<PointType>::open(const std::string &uri, const std::string &calib_file, bool sync, bool upsample, bool average_intensity) {
	if(isOpened())
		return 1;

	laser_num_ = calib_file.empty() ? 32 : 64;

	if(boost::filesystem::exists(uri) || uri.empty()) {
		// Relative path or Absolute path
		//grabber_ = new pcl::HDLGrabber(calib_file, uri);
		grabber_ = new HDLGrabber(calib_file, uri);
	} else {
		// "xxx.xxx.xxx.xxx:xxxx"
		std::vector<std::string> v;
		std::string ip(uri);
		unsigned short port(2368);	// Default
		boost::algorithm::split(v, uri, boost::is_any_of(":"));
		if(v.size() >= 2) {
			ip = v[0];
			port = std::atoi(v[1].c_str());
		}
		//grabber_ = new pcl::HDLGrabber(boost::asio::ip::address::from_string(ip), port, calib_file);
		grabber_ = new HDLGrabber(boost::asio::ip::address::from_string(ip), port, calib_file);
	}
	boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind(&HDLManager<PointType>::cloudCallback_, this, _1);
	cloud_connection_ = grabber_->registerCallback(cloud_cb);
	setSyncMode(sync);
	setUpsampleMode(upsample);
	useAverageIntensityForUpsampling(average_intensity);

	return 0;
}

template<class PointType>
int HDLManager<PointType>::close() {
	if(!isOpened())
		return 1;

	if(isRunning()) {
		if(isDebugMode()) { std::cout << "Stopping HDLGrabber..."; }
		grabber_->stop();
		if(isDebugMode()) { std::cout << "done\n"; }
	}

	if(isDebugMode()) { std::cout << "Disconnecting signals..."; }
	cloud_connection_.disconnect();
	if(isDebugMode()) { std::cout << "done\nDeleting HDLGrabber..."; }
	delete grabber_;
	grabber_ = nullptr;
	if(isDebugMode()) { std::cout << "done\n"; }

	return 0;
}

template<class PointType>
int HDLManager<PointType>::start() {
	if(isRunning())
		return 1;
	grabber_->start();
	return 0;
}

template<class PointType>
int HDLManager<PointType>::stop() {
	if(!isOpened()) {
		return 1;
	} else if(isRunning()) {
		grabber_->stop();
		return 0;
	}
	return 2;
}

template<class PointType>
int HDLManager<PointType>::getPointCloud(CloudConstPtr &cloud) {
	if(!cloud_mutex_.try_lock()) {
		if(isDebugMode())
			std::cerr << "ERROR: Failed to get point cloud" << std::endl;
		return 1;
	}

	cloud = cloud_;
	CloudConstPtr new_cloud;
	cloud_.swap(new_cloud);
	cloud_mutex_.unlock();

	return 0;
}

#endif