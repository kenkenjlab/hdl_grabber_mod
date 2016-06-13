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
#include <time.h>

// ------------------------------------------
//		Declarations
// ------------------------------------------

typedef pcl::PointXYZI PointT;

std::string createDateString(time_t time_data);
std::string createTimeString(time_t time_data);
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);
std::vector<std::vector<int>> clusterLaser(const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices);

// ------------------------------------------
//		Global variables
// ------------------------------------------

HDLLaserManager<PointT>::CloudConstPtr cloud;

// ------------------------------------------
//		Functions
// ------------------------------------------

int main (int argc, char ** argv) {
	HDLLaserManager<PointT>::CloudPtr cloud_filtered(new HDLLaserManager<PointT>::Cloud);

	// (1) Open HDL
	std::string hdlCalibration, pcapFile;
	pcl::console::parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
	pcl::console::parse_argument (argc, argv, "-pcapFile", pcapFile);
	HDLLaserManager<PointT> hdl_man;
	if(hdl_man.open(pcapFile, hdlCalibration)) {
		std::cout << "ERROR: Failed to open!" << std::endl;
		return 1;
	}

	// (2) Create viewer
	int v0(0), v1(1);
	pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_intensity ("intensity"), color_filtered (cloud_filtered, "intensity");
	pcl::visualization::PCLVisualizer viewer("HDL Viewer");
	viewer.addCoordinateSystem (1.0, "reference");
	viewer.setBackgroundColor (0, 0, 0);
	viewer.initCameraParameters ();
	viewer.setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	viewer.setCameraClipDistances (0.0, 50.0);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v0);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v1);
	viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
	
	// (3) Start iteration
	hdl_man.start();
	while(!viewer.wasStopped()) {

		// (a) See if we can get a cloud
		std::vector<std::vector<int>> indices_table;
		if(hdl_man.getPointCloud(cloud, indices_table)) {
			std::cout << "Failed to get a cloud" << std::endl;
		}
		
		// (b) Visualize
		if(cloud) {
			int cluster_idx = 0;
			cloud_filtered->clear();
			BOOST_FOREACH(const std::vector<int> &indices, indices_table) {
				std::vector<std::vector<int>> laser_clusters = clusterLaser(*cloud, indices);
				BOOST_FOREACH(const std::vector<int> &inliers, laser_clusters) {
					if(inliers.size() < 100)
						continue;
					BOOST_FOREACH(int i, inliers) {
						PointT p = (*cloud)[i];
						if(std::fabs(p.y) < 1.0 && p.x < 2.0 && p.x > -1.0 && std::fabs(p.z) < 0.7)
							continue;

						p.intensity = cluster_idx;
						cloud_filtered->push_back(p);
					}
					cluster_idx++;
				}
			}

			color_intensity.setInputCloud(cloud);
			if(!viewer.updatePointCloud (cloud, color_intensity, "original")) {
				viewer.addPointCloud (cloud, color_intensity, "original", v0);
			}
			if(!viewer.updatePointCloud (cloud_filtered, color_filtered, "filtered")) {
				viewer.addPointCloud (cloud_filtered, color_filtered, "filtered", v1);
			}

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


std::vector<std::vector<int>> clusterLaser(const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices) {
	bool is_first_point_intsec = false;
	std::vector<std::vector<int>> clusters(1);
	pcl::PointCloud<PointT>::Ptr cloud_on_plane(new pcl::PointCloud<PointT>);
	pcl::copyPointCloud(cloud, indices, *cloud_on_plane);

	for(int i = 0; i < cloud_on_plane->size(); i++) {
		int prev_i = ((i == 0) ? cloud_on_plane->size() : i) - 1;
		int next_i = (i == cloud_on_plane->size() - 1) ? 0 : (i + 1);
					
		const PointT &pp = (*cloud_on_plane)[prev_i];
		const PointT &p  = (*cloud_on_plane)[i];
		const PointT &pn = (*cloud_on_plane)[next_i];

/*
		Eigen::Vector2f vp = pp.getArray3fMap().head<2>(), v = p.getArray3fMap().head<2>(), vn = pn.getArray3fMap().head<2>();
		Eigen::Vector2f v1 = vp - v, v2 = vn - v;

		float distance = v1.norm();
		float angle = std::acos(v1.normalized().dot(v2.normalized()));
*/
		Eigen::Vector3f vp = pp.getArray3fMap(), v = p.getArray3fMap(), vn = pn.getArray3fMap();
		float distance = std::fabs(v.norm() - vp.norm());

		if(distance > 0.5) {
			if(i != 0) {
				clusters.push_back(std::vector<int>());
			} else {
				is_first_point_intsec = true;
			}
		}
		clusters.back().push_back(indices[i]);
	}

	// Integrate last cluster if same
	if(!is_first_point_intsec) {
		std::vector<int> last_cluster = clusters.back();
		clusters.resize(clusters.size() - 1);
		BOOST_FOREACH(int i, last_cluster)
			clusters.front().push_back(i);
	}

	return clusters;
}