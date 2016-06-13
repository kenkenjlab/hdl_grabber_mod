#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>


template<class PointType>
class SimpleHDLViewer {

public:
	typedef pcl::PointCloud<PointType> Cloud;
	typedef typename Cloud::ConstPtr CloudConstPtr;

	SimpleHDLViewer (pcl::HDLGrabber& grabber, pcl::visualization::PointCloudColorHandler<PointType> &handler) :
		cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL HDL Cloud")),
			grabber_ (grabber),
			handler_ (handler)
			{}

	void cloud_callback (const CloudConstPtr& cloud) {
		boost::mutex::scoped_lock lock (cloud_mutex_);
		cloud_ = cloud;
	}

	void run () {
		cloud_viewer_->addCoordinateSystem (3.0, "reference");
		cloud_viewer_->setBackgroundColor (0, 0, 0);
		cloud_viewer_->initCameraParameters ();
		cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
		cloud_viewer_->setCameraClipDistances (0.0, 50.0);

		boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind (&SimpleHDLViewer::cloud_callback, this, _1);
		boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);

		/*
		for(int i = 1; i < 32; i += 2) {
			pcl::RGB rgb;
			rgb.g = i * 8;
			grabber_.setLaserColorRGB(rgb, i);
		}

		for(int i = 0; i < 32; i += 2) {
			pcl::RGB rgb;
			rgb.b = i * 8;
			grabber_.setLaserColorRGB(rgb, i);
		}

		pcl::RGB rgb;
		rgb.r = 255;
		grabber_.setLaserColorRGB(rgb, 30);
		rgb.g = 255;
		grabber_.setLaserColorRGB(rgb, 31);
		*/

		grabber_.start ();

		while (!cloud_viewer_->wasStopped ()) {
			CloudConstPtr cloud;

			// See if we can get a cloud
			if (cloud_mutex_.try_lock ()) {
				cloud_.swap (cloud);
				cloud_mutex_.unlock ();
			}

			if (cloud) {
				unsigned int timestamp_upper = cloud->header.stamp >> 32;	// time()
				unsigned int timestamp_lower = cloud->header.stamp & 0xffffffff;	// microseconds from the top of the hour
				int minutes = timestamp_lower / 60000000;
				float seconds = (float)timestamp_lower / 1000000 - minutes * 60;
				std::printf("%d pts, timestamp: %dm %fs (%u)\n", cloud->size(), minutes, seconds, timestamp_lower);
				handler_.setInputCloud (cloud);
				if (!cloud_viewer_->updatePointCloud<PointType> (cloud, handler_, "HDL"))
					cloud_viewer_->addPointCloud<PointType> (cloud, handler_, "HDL");

				cloud_viewer_->spinOnce ();
			}

			if (!grabber_.isRunning ()) {
				std::cerr << "Grabber not running\n";
				cloud_viewer_->spin ();
				break;
			}

			boost::this_thread::sleep (boost::posix_time::microseconds (100));
		}

		grabber_.stop ();

		cloud_connection.disconnect ();
	}

private:
	CloudConstPtr cloud_;
	pcl::HDLGrabber& grabber_;
	boost::mutex cloud_mutex_;
	pcl::visualization::PCLVisualizer::Ptr cloud_viewer_;
	pcl::visualization::PointCloudColorHandler<PointType> &handler_;

};

int main (int argc, char ** argv) {
	std::string hdlCalibration, pcapFile;
	if(argc > 1)
		pcapFile = argv[1];
	pcl::console::parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
	pcl::console::parse_argument (argc, argv, "-pcapFile", pcapFile);

	pcl::HDLGrabber grabber (hdlCalibration, pcapFile);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("intensity");
	SimpleHDLViewer<pcl::PointXYZI> v (grabber, color_handler);
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> color_handler;
	//SimpleHDLViewer<pcl::PointXYZRGBA> v (grabber, color_handler);
	v.run ();
	return (0);
}