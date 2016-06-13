#### CMake Settings ####

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_policy(SET CMP0015 OLD)	#// Do not add relative path if the designated path is not exist
add_definitions(-DUNICODE -D_UNICODE)	#// Set Unicode, not MultiBytes

set(PCAP_INC_DIRS "C:/PCL/WpdPack/Include")
set(PCAP_LIB_DIRS "C:/PCL/WpdPack/Lib")
set(PCAP_LIB wpcap.lib)


#### Solution Settings ####

project(hdl_grabber_mod)


#### Includes and Libraries ####

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4)

include_directories(${PCL_INCLUDE_DIRS} ${PCAP_INC_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${PCAP_LIB_DIRS})
add_definitions(${PCL_DEFINITIONS})


#### Required files ####

set(HDL_GRABBER_MOD hdl_grabber_mod.h hdl_grabber_mod.cpp)
set(HDL_MANAGER hdl_manager.hpp ${HDL_GRABBER_MOD})
set(HDL_LASER_MANAGER hdl_laser_manager.hpp ${HDL_MANAGER})
set(HDL_UPSAMPLER hdl_upsampler.hpp ${HDL_MANAGER})
set(HDL_CLUSTERER hdl_clusterer.hpp ${HDL_MANAGER})
set(HDL_RANSAC_CLUSTERER hdl_ransac_clusterer.hpp ${HDL_MANAGER})


#### Projects Settings ####

add_executable (simple_hdl_visualize_test simple_hdl_visualize_test.cpp)
target_link_libraries (simple_hdl_visualize_test ${PCL_LIBRARIES})

add_executable (hdl_manager simple_hdl_manager.cpp ${HDL_MANAGER})
target_link_libraries (hdl_manager ${PCL_LIBRARIES} ${PCAP_LIB})

add_executable (laser_viewer laser_viewer.cpp ${HDL_LASER_MANAGER})
target_link_libraries (laser_viewer ${PCL_LIBRARIES} ${PCAP_LIB})

add_executable (laser_viewer2 laser_viewer2.cpp ${HDL_LASER_MANAGER})
target_link_libraries (laser_viewer2 ${PCL_LIBRARIES} ${PCAP_LIB})

add_executable (upsampling upsampling.cpp ${HDL_UPSAMPLER})
target_link_libraries (upsampling ${PCL_LIBRARIES} ${PCAP_LIB})

add_executable (clustering clustering.cpp ${HDL_CLUSTERER})
target_link_libraries (clustering ${PCL_LIBRARIES} ${PCAP_LIB})

add_executable (laser_ransac laser_ransac.cpp ${HDL_LASER_MANAGER})
target_link_libraries (laser_ransac ${PCL_LIBRARIES} ${PCAP_LIB})

add_executable (small_cluster_removal small_cluster_removal.cpp ${HDL_LASER_MANAGER})
target_link_libraries (small_cluster_removal ${PCL_LIBRARIES} ${PCAP_LIB})

if(DEFINED OpenCV_VERSION)
	add_executable (simple_hdl_grabber_mod simple_hdl_grabber_mod.cpp ${HDL_GRABBER_MOD})
	target_link_libraries (simple_hdl_grabber_mod ${PCL_LIBRARIES} ${PCAP_LIB} ${OpenCV_LIBS})
endif()