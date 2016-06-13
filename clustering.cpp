// ------------------------------------------
//		Includes
// ------------------------------------------

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include "hdl_clusterer.hpp"
#include <time.h>

// ------------------------------------------
//		Global variables
// ------------------------------------------

HDLManager<pcl::PointXYZI>::CloudConstPtr cloud;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

// ------------------------------------------
//		Declarations
// ------------------------------------------

std::string createDateString(time_t time_data);
std::string createTimeString(time_t time_data);
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);


// ------------------------------------------
//		Functions
// ------------------------------------------

int main (int argc, char ** argv) {

	// (1) Open HDL
	std::string hdlCalibration, pcapFile;
	pcl::console::parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
	pcl::console::parse_argument (argc, argv, "-pcapFile", pcapFile);
	HDLClusterer<pcl::PointXYZI> hdl_man;
	if(hdl_man.open(pcapFile, hdlCalibration)) {
		std::cout << "ERROR: Failed to open!" << std::endl;
		return 1;
	}

	// (2) Create viewer
	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("intensity");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_handler(255, 255, 255);
	pcl::visualization::PCLVisualizer viewer("HDL Viewer");
	viewer.addCoordinateSystem (3.0, "reference");
	viewer.setBackgroundColor (0, 0, 0);
	viewer.initCameraParameters ();
	viewer.setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	viewer.setCameraClipDistances (0.0, 50.0);
	viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	// (3) Start iteration
	hdl_man.start();
	while(!viewer.wasStopped()) {

		// (a) See if we can get a cloud
		if(hdl_man.getPointCloud(cloud, *colored_cloud)) {
			std::cout << "Failed to get a cloud" << std::endl;
		}
		
		// (b) Visualize
		if(cloud) {
			std::printf("%d points, %d special points\n", cloud->size(), colored_cloud->size());
			color_handler.setInputCloud(cloud);
			if(!viewer.updatePointCloud (cloud, color_handler, "HDL"))
				viewer.addPointCloud (cloud, color_handler, "HDL");
			if(!viewer.updatePointCloud (colored_cloud, "clusters"))
				viewer.addPointCloud (colored_cloud, "clusters");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "clusters");
			viewer.spinOnce ();
		}

		// (c) Stop reading next cloud if last cloud
		if(!hdl_man.isRunning()) {
			std::cout << "Reached the last frame" << std::endl;
			viewer.spin();
		}

		boost::this_thread::sleep (boost::posix_time::microseconds (100));
	}

	// (4) Close
	hdl_man.stop();
	hdl_man.close();
	
	return 0;

}

std::string createDateString(time_t time_data) {
	struct tm *pnow = localtime(&time_data);
	std::stringstream ss;
	ss << (pnow->tm_year + 1900)
		<< std::setw(2) << std::setfill('0') << (pnow->tm_mon + 1)
		<< std::setw(2) << std::setfill('0') << pnow->tm_mday;
	return ss.str();
}

std::string createTimeString(time_t time_data) {
	struct tm *pnow = localtime(&time_data);
	std::stringstream ss;
	ss << std::setw(2) << std::setfill('0') << pnow->tm_hour
			<< std::setw(2) << std::setfill('0') << pnow->tm_min
			<< std::setw(2) << std::setfill('0') << pnow->tm_sec;
	return ss.str();
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	if(event.getKeySym() == "n" && event.keyDown()) {
		time_t now = time(0);
		std::string file_name = createDateString(now) + "_" + createTimeString(now) + ".pcd";
		std::cerr << "Saving as \"" << file_name << "\"...";
		pcl::io::savePCDFile(file_name, *cloud);
		std::cerr << "done" << std::endl;
	}
}
