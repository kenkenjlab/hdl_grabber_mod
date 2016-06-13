// ------------------------------------------
//		Includes
// ------------------------------------------

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include "hdl_laser_manager.hpp"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <time.h>
#include "color_sample.hpp"

// ------------------------------------------
//		Declarations
// ------------------------------------------

std::string createDateString(time_t time_data);
std::string createTimeString(time_t time_data);
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);
template<class PointType> void ransacClustering(const pcl::PointCloud<PointType> &cloud, const std::vector<int> &indices, std::vector<std::vector<int>> &clusters);

typedef pcl::PointXYZI PointT;

// ------------------------------------------
//		Global variables
// ------------------------------------------

HDLLaserManager<PointT>::CloudConstPtr cloud;
pcl::PointCloud<PointT>::Ptr cloud_on_laser(new pcl::PointCloud<PointT>);

// ------------------------------------------
//		Functions
// ------------------------------------------

int main (int argc, char ** argv) {
	// (1) Open HDL
	int laser_idx = 16;
	std::string hdlCalibration, pcapFile;
	pcl::console::parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
	pcl::console::parse_argument (argc, argv, "-pcapFile", pcapFile);
	HDLLaserManager<PointT> hdl_man;
	if(hdl_man.open(pcapFile, hdlCalibration)) {
		std::cout << "ERROR: Failed to open!" << std::endl;
		return 1;
	}

	// (2) Create viewer
	int v1 (0);
	int v2 (1);
	int v3 (2);
	int v4 (3);
	int v5 (4);
	pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler1 (cloud_on_laser, "intensity");
	pcl::visualization::PCLVisualizer viewer("HDL Viewer");
	viewer.addCoordinateSystem (1.0, "reference");
	viewer.setBackgroundColor (0, 0, 0);
	viewer.initCameraParameters ();
	viewer.setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	viewer.setCameraClipDistances (0.0, 50.0);
	viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	// (3) Start iteration
	hdl_man.start();
	while(!viewer.wasStopped()) {

		// (a) See if we can get a cloud
		std::vector<std::vector<int>> indices_table;
		if(hdl_man.getPointCloud(cloud, indices_table)) {
			std::cout << "Failed to get a cloud" << std::endl;
		}
		
		// (b) If successfully acquired
		if(cloud && indices_table.size()) {
			// (A) For each laser,
			std::vector<std::vector<std::vector<int>>> clusters_table;
			int cluster_size = 0;
			BOOST_FOREACH(const std::vector<int>& indices, indices_table) {
				std::vector<std::vector<int>> clusters;
				ransacClustering(*cloud, indices, clusters);
				std::printf("[%d] %d points -> %d clusters\n", clusters_table.size(), indices.size(), clusters.size());
				clusters_table.push_back(clusters);
				cluster_size += clusters.size();
			}
			std::cout << "size: " << cluster_size << std::endl;
			std::cout << "-----" << std::endl;
			
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


template<class PointType>
void ransacClustering(const pcl::PointCloud<PointType> &cloud, const std::vector<int> &indices, std::vector<std::vector<int>> &clusters) {
	// (1) Project on XY plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_on_plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud, *cloud_on_plane);
	BOOST_FOREACH(pcl::PointXYZ &p, *cloud_on_plane) { p.z = 0.0; }
	int original_size = cloud_on_plane->size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_on_plane_removed(new pcl::PointCloud<pcl::PointXYZ>);
	
	// (2) Iterate line and circle segmentation alternately
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (true);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);
	seg.setMaxIterations(200);
	pcl::ExtractIndices<pcl::PointXYZ> ext;
	ext.setNegative(true);

	pcl::IndicesPtr indices_tmp(new std::vector<int>);
	*indices_tmp = indices;
	while(cloud_on_plane->size() > original_size * 0.3) {
		std::cout << "A" << indices_tmp->size() << std::endl;
		seg.setInputCloud (cloud_on_plane);
		seg.setIndices(indices_tmp);
		seg.setModelType (pcl::SACMODEL_LINE);
		seg.segment (*inliers, *coefficients);
		clusters.push_back(inliers->indices);
		
		std::cout << "B" << indices_tmp->size() << std::endl;
		ext.setInputCloud(cloud_on_plane);
		ext.setIndices(inliers);
		ext.filter(*indices_tmp);

		std::cout << "C" << indices_tmp->size() << std::endl;
		seg.setInputCloud (cloud_on_plane);
		seg.setIndices(indices_tmp);
		seg.setModelType (pcl::SACMODEL_CIRCLE2D);
		seg.segment (*inliers, *coefficients);
		clusters.push_back(inliers->indices);

		std::cout << "D" << indices_tmp->size() << std::endl;
		ext.setIndices(inliers);
		ext.filter(*indices_tmp);
	}
}