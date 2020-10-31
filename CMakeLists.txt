cmake_minimum_required(VERSION 2.8.3)
project(lidar_stereo_ros)

#get_filename_component(EXTERNAL_LIBRARIES ../../ ABSOLUTE)
find_package(PCL 1.9 REQUIRED PATHS ../pcl_1_9/install_dir/share/pcl-1.9 NO_DEFAULT_PATH )
find_package(Boost) 

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS}) 
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (test_1 test_1.cpp)
#target_link_libraries (test_1 ${PCL_LIBRARIES})

#add_executable (test_4 test_4.cpp)
#target_link_libraries (test_4 ${PCL_LIBRARIES})


#add_executable (test_5 test_5.cpp)
#target_link_libraries (test_5 pcl_common pcl_io pcl_registration pcl_keypoints pcl_sample_consensus pcl_visualization pcl_search pcl_features pcl_kdtree pcl_surface pcl_segmentation boost_system flann_cpp pcl_filters pcl_segmentation pcl_kdtree pcl_surface pcl_octree pcl_registration ) 


add_executable (test_6 test_6.cpp)
target_link_libraries (test_6 pcl_common pcl_io pcl_registration pcl_keypoints pcl_sample_consensus pcl_visualization pcl_search pcl_features pcl_kdtree pcl_surface pcl_segmentation boost_system flann_cpp pcl_filters pcl_segmentation pcl_kdtree pcl_surface pcl_octree pcl_registration ) 

#add_executable (test_icp test_simple_icp.cpp)
#target_link_libraries (test_icp ${PCL_LIBRARIES})

#add_executable(test_2 test_2.cpp)
#target_link_libraries(test_2 ${PCL_LIBRARIES} pcl_common pcl_io pcl_registration pcl_keypoints pcl_sample_consensus pcl_visualization pcl_search pcl_features pcl_kdtree pcl_surface pcl_segmentation)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
#add_executable(test_3 test_3_voxel.cpp)
#target_link_libraries(test_3 ${PCL_LIBRARIES} ${Boost_LIBRARIES} pcl_common pcl_io pcl_registration pcl_keypoints pcl_sample_consensus pcl_visualization pcl_search pcl_features pcl_kdtree pcl_surface pcl_segmentation)
