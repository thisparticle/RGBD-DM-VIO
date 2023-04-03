PROJECT(d2vio)
CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
# set(BUILD_TYPE Release)
IF(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()
set(BUILD_TYPE RelWithDebInfo)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(SuiteParse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread filesystem chrono serialization date_time timer regex)
find_package(Pangolin 0.2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTSAM REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp
	rosbag
	sensor_msgs
	cv_bridge
	image_transport
	message_filters
	message_runtime
	message_generation
)

IF(${Boost_VERSION} GREATER_EQUAL 106500)
    message("Building with stacktrace support.")
	set(STACKTRACE_LIBRARIES dl)
	set(STACKTRACE_DEFINES "-DSTACKTRACE -DBOOST_STACKTRACE_USE_ADDR2LINE")
ENDIF()

# catkin_package()
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS}"
)
set(CMAKE_CXX_FLAGS_DEBUG
    "-O2 -g -fno-omit-frame-pointer -DEIGEN_INITIALIZE_MATRICES_WITH_NAN -DDEBUG ${STACKTRACE_DEFINES}"
)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
    "-O3 -g -fno-omit-frame-pointer ${STACKTRACE_DEFINES}"
)
set(CMAKE_CXX_FLAGS_RELEASE
    "-O3 -DNDEBUG"
)

if (MSVC)
     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif (MSVC)

add_message_files(
  FILES
  DMVIOPoseMsg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)


catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs roscpp sensor_msgs message_runtime
)


include_directories(
	src
  src/dso
	thirdparty/Sophus
	thirdparty/sse2neon
	${EIGEN3_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Pangolin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${CSPARSE_INCLUDE_DIR} 
	${CHOLMOD_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
) 

add_library(d2vio_ros_lib
	src/dso/FullSystem/FullSystem.cpp
	src/dso/FullSystem/FullSystemOptimize.cpp
	src/dso/FullSystem/FullSystemOptPoint.cpp
	src/dso/FullSystem/FullSystemDebugStuff.cpp
	src/dso/FullSystem/FullSystemMarginalize.cpp
	src/dso/FullSystem/Residuals.cpp
	src/dso/FullSystem/CoarseInitializer.cpp
	src/dso/FullSystem/CoarseTracker.cpp
	src/dso/FullSystem/ImmaturePoint.cpp
	src/dso/FullSystem/HessianBlocks.cpp
	src/dso/FullSystem/PixelSelector2.cpp
	src/dso/OptimizationBackend/EnergyFunctional.cpp
	src/dso/OptimizationBackend/AccumulatedTopHessian.cpp
	src/dso/OptimizationBackend/AccumulatedSCHessian.cpp
	src/dso/OptimizationBackend/EnergyFunctionalStructs.cpp
	src/dso/util/settings.cpp
	src/dso/util/Undistort.cpp
	src/dso/util/globalCalib.cpp
	src/dso/IOWrapper/Pangolin/KeyFrameDisplay.cpp
	src/dso/IOWrapper/Pangolin/PangolinDSOViewer.cpp
	src/dso/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
	src/dso/IOWrapper/OpenCV/ImageRW_OpenCV.cpp
  # 
  src/IMU/IMUIntegration.cpp
  src/GTSAMIntegration/Sim3GTSAM.cpp
  src/IMUInitialization/GravityInitializer.cpp
  src/IMU/IMUTypes.cpp
  src/IMU/IMUSettings.cpp
  src/util/TimeMeasurement.cpp
  src/util/SettingsUtil.cpp
  src/GTSAMIntegration/BAGTSAMIntegration.cpp
  src/IMU/CoarseIMULogic.cpp
  src/IMU/BAIMULogic.cpp
  src/GTSAMIntegration/PoseTransformation.cpp
  src/GTSAMIntegration/Marginalization.cpp
  src/GTSAMIntegration/PoseTransformationIMU.cpp
  src/GTSAMIntegration/PoseTransformationFactor.cpp
  src/IMUInitialization/CoarseIMUInitOptimizer.cpp
  src/IMUInitialization/IMUInitializer.cpp
  src/IMU/IMUUtils.cpp
  src/IMUInitialization/IMUInitSettings.cpp
  src/GTSAMIntegration/GTSAMUtils.cpp
  src/GTSAMIntegration/DelayedMarginalization.cpp
  src/IMUInitialization/PoseGraphBundleAdjustment.cpp
  src/GTSAMIntegration/FEJValues.cpp
  src/IMUInitialization/IMUInitializerStates.cpp
  src/IMUInitialization/IMUInitializerLogic.cpp
  src/IMUInitialization/IMUInitializerTransitions.cpp
  src/GTSAMIntegration/AugmentedScatter.cpp
  src/live/FrameContainer.cpp
  src/live/IMUInterpolator.cpp
  src/util/MainSettings.cpp
  src/live/FrameSkippingStrategy.cpp
  src/live/DatasetSaver.cpp
)


add_executable(d2vio_ros_node src/main_ros.cpp src/ROSOutputWrapper.cpp)

target_link_libraries(d2vio_ros_node
	d2vio_ros_lib 
	${catkin_LIBRARIES} 
	${BOOST_THREAD_LIBRARY} 
	${Pangolin_LIBRARIES}
  gtsam 
  ${YAML_CPP_LIBRARIES} 
  ${STACKTRACE_LIBRARIES} 
	${OpenCV_LIBS} 
	boost_system boost_thread cxsparse)

add_executable(d2vio_node src/main_no_ros.cpp)

target_link_libraries(d2vio_node
	d2vio_ros_lib 
	${catkin_LIBRARIES} 
	${BOOST_THREAD_LIBRARY} 
	${Pangolin_LIBRARIES}
  gtsam 
  ${YAML_CPP_LIBRARIES} 
  ${STACKTRACE_LIBRARIES} 
	${OpenCV_LIBS} 
	boost_system boost_thread cxsparse)

# add_subdirectory(test)
set(DMVIO_LINKED_LIBRARIES boost_system cxsparse ${BOOST_THREAD_LIBRARY} ${LIBZIP_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} gtsam ${YAML_CPP_LIBRARIES} ${STACKTRACE_LIBRARIES})