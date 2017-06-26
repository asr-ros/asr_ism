cmake_minimum_required(VERSION 2.4.6)
project(asr_ism)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# We need c++11 support, because some functions are written in c++11
#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
add_definitions(-Wall -std=gnu++0x -ggdb)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  asr_msgs
  visualization_msgs
  roslib
  asr_lib_ism
  asr_ism_visualizations
  dynamic_reconfigure
  asr_lib_pose_prediction_ism
  asr_object_database
  asr_rapidxml
)
generate_dynamic_reconfigure_options(
  cfg/recognizer.cfg
  cfg/recordGen.cfg
)

#Add include directories to the build.
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PCL_COMMON_INCLUDE_DIR})
include_directories(${PCL_IO_INCLUDE_DIR})

# Added libism as include_dir to provide the Recognizer.hpp to another package.
catkin_package(    
    CATKIN_DEPENDS roscpp tf asr_msgs visualization_msgs roslib asr_lib_ism asr_ism_visualizations asr_lib_pose_prediction_ism
    DEPENDS Boost asr_rapidxml EIGEN3
)

add_executable(recordViewer src/recordViewer.cpp)
target_link_libraries(recordViewer 
  ism
  ${catkin_LIBRARIES}
)

add_executable(fake_data_publisher src/fake_data_publisher.cpp)
target_link_libraries(fake_data_publisher 
  ism
  ${catkin_LIBRARIES}
)

add_executable(modelViewer src/modelViewer.cpp)
target_link_libraries(modelViewer 
  ism
  ${catkin_LIBRARIES}
)

add_executable(voteViewer src/voteViewer.cpp)
target_link_libraries(voteViewer 
  ism
  ${catkin_LIBRARIES}
)

add_executable(recorder 
  src/recorder.cpp
  src/ism_helper.cpp
  src/asr_ism/scene_configurator.cpp
  src/asr_ism/scene_configurator_data.cpp
)

target_link_libraries(recorder 
  ism 
  ${catkin_LIBRARIES}
)

add_executable(recognizer 
  src/recognizer.cpp
  src/ism_helper.cpp
  src/asr_ism/scene_configurator.cpp
  src/asr_ism/scene_configurator_data.cpp
)

target_link_libraries(recognizer 
  ism
  asr_lib_pose_prediction_ism
  ${catkin_LIBRARIES}
)

# make sure configure headers are built before any node using them
add_dependencies(recognizer ${PROJECT_NAME}_gencfg)

add_executable(trainer
  src/trainer.cpp
  src/ism_helper.cpp
)

target_link_libraries(trainer 
  ism
  ${catkin_LIBRARIES}
)

add_executable(combinatorialTrainer src/combinatorialTrainer.cpp)

target_link_libraries(combinatorialTrainer
  ism
  ${catkin_LIBRARIES}
)

add_executable(recordGen src/recordGen.cpp)
target_link_libraries(recordGen 
  ism
  ${catkin_LIBRARIES}
  pcl_filters
)

add_executable(object_configuration_generator src/object_configuration_generator.cpp)
target_link_libraries(object_configuration_generator
  ism
  ${catkin_LIBRARIES}
)

#tools
add_executable(data_merger src/data_merger.cpp)
target_link_libraries(data_merger
  ism
  ${catkin_LIBRARIES}
)

add_executable(data_cleaner src/data_cleaner.cpp)
target_link_libraries(data_cleaner
  ism
  ${catkin_LIBRARIES}
)

add_executable(marker_rotator src/marker_rotator.cpp)
target_link_libraries(marker_rotator
  ism
  ${catkin_LIBRARIES}
)

add_executable(pose_interpolator src/pose_interpolator.cpp)
target_link_libraries(pose_interpolator
  ism
  ${catkin_LIBRARIES}
)

add_executable(recorded_objects_transformer src/recorded_objects_transformer.cpp)
target_link_libraries(recorded_objects_transformer
  ism
  ${catkin_LIBRARIES}
)

add_executable(rotation_invariant_objects_rotator src/rotation_invariant_objects_rotator.cpp)
target_link_libraries(rotation_invariant_objects_rotator
  ism
  ${catkin_LIBRARIES}
)
