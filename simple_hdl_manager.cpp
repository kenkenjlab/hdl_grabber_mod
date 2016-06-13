// ------------------------------------------
//		Includes
// ------------------------------------------

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include "hdl_manager.hpp"
#include <time.h>

// ------------------------------------------
//		Global variables
// ------------------------------------------

HDLManager<pcl::PointXYZI>::CloudConstPtr cloud;


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
	if(argc > 1)
		pcapFile = argv[1];
	unsigned int upsample_num(1);
	float upsample_max_sq_dist_threshold(0.25);

	pcl::console::parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
	pcl::console::parse_argument (argc, argv, "-pcapFile", pcapFile);
	bool sync = pcl::console::find_argument (argc, argv, "-sync") != -1;
	bool upsample = pcl::console::find_argument (argc, argv, "-upsample") != -1;
	pcl::console::parse_argument (argc, argv, "-upsample_num", upsample_num);
	pcl::console::parse_argument (argc, argv, "-upsample_max_sq_dist_threshold", upsample_max_sq_dist_threshold);
	bool average_intensity = pcl::console::find_argument (argc, argv, "-average_intensity") != -1;
	if(sync) { std::printf("* Sync mode\n"); }
	if(upsample) { std::printf("* Upsample mode (max %fm^2, +num %d)\n", upsample_max_sq_dist_threshold, upsample_num); }
	if(average_intensity) { std::printf("* Use average intensity\n"); }

	HDLManager<pcl::PointXYZI> hdl_man;
	if(hdl_man.open(pcapFile, hdlCalibration, sync, upsample, average_intensity)) {
		std::cout << "ERROR: Failed to open!" << std::endl;
		return 1;
	}
	hdl_man.setUpsampleMaxSquaredDistanceThreshold(upsample_max_sq_dist_threshold);
	hdl_man.setUpsampleNumOfPoints(upsample_num);

	// (2) Create viewer
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("intensity");
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
		if(sync) { hdl_man.acquire(); }

		// (a) See if we can get a cloud
		if(hdl_man.getPointCloud(cloud)) {
			std::cout << "Failed to get a cloud" << std::endl;
		}

		// (b) Visualize
		if(cloud) {
			std::printf("%d points\n", cloud->size());
			color_handler.setInputCloud(cloud);
			if(!viewer.updatePointCloud (cloud, color_handler, "HDL"))
				viewer.addPointCloud (cloud, color_handler, "HDL");
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
