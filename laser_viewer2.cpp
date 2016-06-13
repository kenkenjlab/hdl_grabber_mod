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

std::string createDateString(time_t time_data);
std::string createTimeString(time_t time_data);
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

typedef pcl::PointXYZI PointT;

// ------------------------------------------
//		Global variables
// ------------------------------------------

HDLLaserManager<PointT>::CloudConstPtr cloud;
pcl::PointCloud<PointT>::Ptr cloud_on_plane(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_intersection(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_segmented(new pcl::PointCloud<PointT>);

// ------------------------------------------
//		Functions
// ------------------------------------------

int main (int argc, char ** argv) {
	// (1) Open HDL
	int laser_idx = 16;
	std::string hdlCalibration, pcapFile;
	pcl::console::parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
	pcl::console::parse_argument (argc, argv, "-pcapFile", pcapFile);
	pcl::console::parse_argument (argc, argv, "-laserIndex", laser_idx);
	HDLLaserManager<PointT> hdl_man;
	if(hdl_man.open(pcapFile, hdlCalibration)) {
		std::cout << "ERROR: Failed to open!" << std::endl;
		return 1;
	}

	// (2) Create viewer
	int v0(0), v1(1);
	pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_intensity (cloud_on_plane, "intensity"), color_cluster(cloud_segmented, "intensity");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_red(cloud_intersection, 255, 0, 0);
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
			cloud_on_plane->clear();
			cloud_segmented->clear();
			if(indices_table.size() > laser_idx + 1) {
				const std::vector<int> &indices = indices_table[laser_idx];
				BOOST_FOREACH(int i, indices) {
					PointT p1 = cloud->points[i];
					p1.intensity = i;
					cloud_on_plane->push_back(p1);
				}

				cloud_intersection->clear();
				bool is_first_point_intsec = false;
				int cluster_idx = 0;
				for(int i = 0; i < cloud_on_plane->size(); i++) {
					int prev_i = ((i == 0) ? cloud_on_plane->size() : i) - 1;
					int next_i = (i == cloud_on_plane->size() - 1) ? 0 : (i + 1);
					
					const PointT &pp = (*cloud_on_plane)[prev_i];
					PointT p = (*cloud_on_plane)[i];
					const PointT &pn = (*cloud_on_plane)[next_i];

					Eigen::Vector2f vp = pp.getArray3fMap().head<2>(), v = p.getArray3fMap().head<2>(), vn = pn.getArray3fMap().head<2>();
					Eigen::Vector2f v1 = vp - v, v2 = vn - v;

					float distance = v1.norm();
					float angle = std::acos(v1.normalized().dot(v2.normalized()));

					//if(angle < M_PI * 3 / 8 || v1.norm() > 0.1) {
					if(v1.norm() > 0.1) {
						if(i != 0) {
							cluster_idx++;
						} else {
							is_first_point_intsec = true;
						}
						cloud_intersection->push_back(p);
					}
					p.intensity = cluster_idx;
					cloud_segmented->push_back(p);
				}

				// Integrate first and last cluster if same
				if(!is_first_point_intsec) {
					int last_intensity = cloud_segmented->back().intensity;
					BOOST_FOREACH(PointT &p, *cloud_segmented)
						if(p.intensity == last_intensity) p.intensity = 0;
				}

				std::printf("%d points, %d laser points\n", cloud->size(), cloud_on_plane->size());

				if(!cloud_on_plane->empty()) {
					if(!viewer.updatePointCloud (cloud_on_plane, color_intensity, "laser1"))
						viewer.addPointCloud (cloud_on_plane, color_intensity, "laser1", v0);
					if(!viewer.updatePointCloud (cloud_intersection, color_red, "intersection")) {
						viewer.addPointCloud (cloud_intersection, color_red, "intersection", v0);
						viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "intersection");
					}
					if(!viewer.updatePointCloud (cloud_segmented, color_cluster, "cluster"))
						viewer.addPointCloud (cloud_segmented, color_cluster, "cluster", v1);
					viewer.spinOnce ();
				}
			}
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
