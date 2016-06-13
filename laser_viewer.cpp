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
template <class ScalarT> std::vector<ScalarT> differentiate_(const std::vector<ScalarT> &values);
template <class ScalarT> std::vector<ScalarT> differentiate2_(const std::vector<ScalarT> &values);

typedef pcl::PointXYZI PointT;

// ------------------------------------------
//		Global variables
// ------------------------------------------

HDLLaserManager<PointT>::CloudConstPtr cloud;
pcl::PointCloud<PointT>::Ptr cloud_on_laser(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_on_laser_polar(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_on_laser_polar_1(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_on_laser_polar_2(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_on_laser_polar_3(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_intersection(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_intersection_polar(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_intersection_polar_1(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_intersection_polar_2(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_intersection_polar_3(new pcl::PointCloud<PointT>);

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
	std::cout << "Laser Index: " << laser_idx << std::endl;

	// (2) Create viewer
	int v1 (0);
	int v2 (1);
	int v3 (2);
	int v4 (3);
	int v5 (4);
	pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler1 (cloud_on_laser, "intensity");
	pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler2 (cloud_on_laser_polar, "intensity");
	pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler3 (cloud_on_laser_polar_1, "intensity");
	pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler4 (cloud_on_laser_polar_2, "intensity");
	pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler5 (cloud_on_laser_polar_3, "intensity");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler6(cloud_intersection, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler7(cloud_intersection_polar, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler8(cloud_intersection_polar_1, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler9(cloud_intersection_polar_2, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler10(cloud_intersection_polar_3, 255, 0, 0);
	pcl::visualization::PCLVisualizer viewer("HDL Viewer");
	viewer.addCoordinateSystem (1.0, "reference");
	viewer.setBackgroundColor (0, 0, 0);
	viewer.initCameraParameters ();
	viewer.setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	viewer.setCameraClipDistances (0.0, 50.0);
	viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
	viewer.createViewPort (0.0, 0.0, 0.5, 0.5, v1);
	viewer.createViewPort (0.5, 0.0, 1.0, 0.5, v2);
	viewer.createViewPort (0.0, 0.5, 1.0/3, 1.0, v3);
	viewer.createViewPort (1.0/3, 0.5, 2.0/3, 1.0, v4);
	viewer.createViewPort (2.0/3, 0.5, 1.0, 1.0, v5);
	viewer.addText("Laser" + laser_idx, 10, 10, "text1", v1);
	viewer.addText("Polar ( X: Angle, Y: Distance )", 10, 10, "text2", v2);
	viewer.addText("1st-order diff of Polar", 10, 10, "text3", v3);
	viewer.addText("2nd-order diff of Polar", 10, 10, "text4", v4);
	viewer.addText("3rd-order diff of Polar", 10, 10, "text5", v5);

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
			cloud_on_laser->clear();
			cloud_on_laser_polar->clear();
			std::vector<float> dist, dist_1, dist_2, dist_3;
			if(indices_table.size() > laser_idx + 1) {
				const std::vector<int> &indices = indices_table[laser_idx];
				BOOST_FOREACH(int i, indices) {
					PointT p1 = cloud->points[i];
					p1.intensity = i;	//angle;
					cloud_on_laser->push_back(p1);
					
					float angle = std::atan2(p1.x, p1.y);
					float distance = static_cast<Eigen::Vector3f>(p1.getArray3fMap()).head<2>().norm();	// sqrt(x^2 + y^2)
					PointT p2;
					pcl::copyPoint(p1, p2);
					//p2.x = angle - M_PI_2;
					p2.x = ((float)i / indices.size()) - 0.5;
					p2.y = distance;
					p2.z = 0.0;
					cloud_on_laser_polar->push_back(p2);
					dist.push_back(distance);
				}

				for(int i = 0; i < dist.size(); i++) {
					int prev_i = ((i == 0) ? dist.size() : i) - 1;
					int next_i = (i == dist.size() - 1) ? 0 : (i + 1);
					dist[i] = (dist[prev_i] + dist[i] + dist[next_i]) / 3;
				}

				pcl::copyPointCloud(*cloud_on_laser_polar, *cloud_on_laser_polar_1);
				pcl::copyPointCloud(*cloud_on_laser_polar, *cloud_on_laser_polar_2);
				pcl::copyPointCloud(*cloud_on_laser_polar, *cloud_on_laser_polar_3);
				cloud_intersection->clear();
				cloud_intersection_polar->clear();
				cloud_intersection_polar_1->clear();
				cloud_intersection_polar_2->clear();
				cloud_intersection_polar_3->clear();

				dist_1 = differentiate2_(dist);
				dist_2 = differentiate2_(dist_1);
				dist_3 = differentiate2_(dist_2);

				for(int i = 0; i < dist.size(); i++) {
					(*cloud_on_laser_polar_1)[i].y = dist_1[i];
					(*cloud_on_laser_polar_2)[i].y = dist_2[i];
					(*cloud_on_laser_polar_3)[i].y = dist_3[i];

					//if(std::fabs(dist_1[i]) > 0.5) {
					if(std::fabs(dist_1[i]) > std::fabs(dist[i]) * 0.05) {
						cloud_intersection->push_back((*cloud_on_laser)[i]);
						cloud_intersection_polar->push_back((*cloud_on_laser_polar)[i]);
						cloud_intersection_polar_1->push_back((*cloud_on_laser_polar_1)[i]);
						cloud_intersection_polar_2->push_back((*cloud_on_laser_polar_2)[i]);
						cloud_intersection_polar_3->push_back((*cloud_on_laser_polar_3)[i]);
					}
				}
				
				std::printf("%d points, %d laser points\n", cloud->size(), cloud_on_laser->size());

				if(!cloud_on_laser->empty()) {
					if(!viewer.updatePointCloud (cloud_on_laser, color_handler1, "laser1"))
						viewer.addPointCloud (cloud_on_laser, color_handler1, "laser1", v1);
					if(!viewer.updatePointCloud (cloud_on_laser_polar, color_handler2, "laser2"))
						viewer.addPointCloud (cloud_on_laser_polar, color_handler2, "laser2", v2);
					if(!viewer.updatePointCloud (cloud_on_laser_polar_1, color_handler3, "laser3"))
						viewer.addPointCloud (cloud_on_laser_polar_1, color_handler3, "laser3", v3);
					if(!viewer.updatePointCloud (cloud_on_laser_polar_2, color_handler4, "laser4"))
						viewer.addPointCloud (cloud_on_laser_polar_2, color_handler4, "laser4", v4);
					if(!viewer.updatePointCloud (cloud_on_laser_polar_3, color_handler5, "laser5"))
						viewer.addPointCloud (cloud_on_laser_polar_3, color_handler5, "laser5", v5);
					if(!viewer.updatePointCloud (cloud_intersection, color_handler6, "laser6")) {
						viewer.addPointCloud (cloud_intersection, color_handler6, "laser6", v1);
						viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "laser6");
					}
					if(!viewer.updatePointCloud (cloud_intersection_polar, color_handler7, "laser7")) {
						viewer.addPointCloud (cloud_intersection_polar, color_handler7, "laser7", v2);
						viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "laser7");
					}
					if(!viewer.updatePointCloud (cloud_intersection_polar_1, color_handler8, "laser8")) {
						viewer.addPointCloud (cloud_intersection_polar_1, color_handler8, "laser8", v3);
						viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "laser8");
					}
					if(!viewer.updatePointCloud (cloud_intersection_polar_2, color_handler9, "laser9")) {
						viewer.addPointCloud (cloud_intersection_polar_2, color_handler9, "laser9", v4);
						viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "laser9");
					}
					if(!viewer.updatePointCloud (cloud_intersection_polar_3, color_handler10, "laser10")) {
						viewer.addPointCloud (cloud_intersection_polar_3, color_handler10, "laser10", v5);
						viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "laser10");
					}
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

template <class ScalarT>
std::vector<ScalarT> differentiate_(const std::vector<ScalarT> &values) {
	int size = values.size();
	std::vector<ScalarT> diffs(size);
	for(int i = 0; i < size; i++) {
		int prev_i = ((i == 0) ? size : i) - 1;
		int next_i = ((i == size - 1) ? 0 : i + 1);
		//diffs[i] = ((values[next_i] - values[i]) + (values[i] - values[prev_i]));
		diffs[i] = values[next_i] - values[prev_i];
	}
	return diffs;
}

template <class ScalarT>
std::vector<ScalarT> differentiate2_(const std::vector<ScalarT> &values) {
	int size = values.size();
	std::vector<ScalarT> diffs(size);
	for(int i = 0; i < size; i++) {
		int prev_i = ((i == 0) ? size : i) - 1;
		diffs[i] = values[i] - values[prev_i];
	}
	return diffs;
}