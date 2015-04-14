cmake_minimum_required(VERSION 2.8.3)
project(find_object_2d)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
             cv_bridge roscpp rospy sensor_msgs std_msgs image_transport genmsg message_filters pcl_ros tf
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ObjectsStamped.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs image_transport message_filters pcl_ros tf
)

###########
## Build ##
###########
SET(PROJECT_VERSION “0.5.1”)
SET(PROJECT_PREFIX find_object)

STRING(REGEX MATCHALL "[0-9]" PROJECT_VERSION_PARTS "${PROJECT_VERSION}")
LIST(GET PROJECT_VERSION_PARTS 0 PROJECT_VERSION_MAJOR)
LIST(GET PROJECT_VERSION_PARTS 1 PROJECT_VERSION_MINOR)
LIST(GET PROJECT_VERSION_PARTS 2 PROJECT_VERSION_PATCH)

ADD_DEFINITIONS(-DPROJECT_PREFIX="${PROJECT_PREFIX}")
ADD_DEFINITIONS(-DPROJECT_VERSION="${PROJECT_VERSION}")

find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork)
FIND_PACKAGE(OpenCV REQUIRED) # tested on 2.3.1

IF(OPENCV_NONFREE_FOUND)
  SET(NONFREE 1)
ELSE()
  SET(NONFREE 0)
ENDIF()
CONFIGURE_FILE(Version.h.in ${PROJECT_SOURCE_DIR}/include_x/${PROJECT_PREFIX}/Version.h)

### Qt Gui stuff ###
SET(headers_ui 
	./include_x/find_object/MainWindow.h
	./include_x/find_object/TcpServer.h
	./include_x/find_object/ObjWidget.h
	./include_x/find_object/Camera.h
	./include_x/find_object/FindObject.h
	./src_x/AddObjectDialog.h
	./src_x/ParametersToolBox.h
	./src_x/RectItem.h
	./src_x/CameraTcpServer.h
	./src/CameraROS.h
	./src/FindObjectROS.h
	./src_x/AboutDialog.h
	./src_x/utilite/UPlot.h
	./src_x/rtabmap/PdfPlot.h
	./src_x/ImageDropWidget.h
)

SET(uis
    ./src_x/ui/mainWindow.ui
    ./src_x/ui/addObjectDialog.ui
    ./src_x/ui/aboutDialog.ui
)

SET(qrc 
    ./src_x/resources.qrc
)

# generate rules for building source files from the resources
QT4_ADD_RESOURCES(srcs_qrc ${qrc})

#Generate .h files from the .ui files
QT4_WRAP_UI(moc_uis ${uis})

#This will generate moc_* for Qt
QT4_WRAP_CPP(moc_srcs ${headers_ui})
### Qt Gui stuff  end###


SET(SRC_FILES 
	./src_x/MainWindow.cpp
	./src_x/AddObjectDialog.cpp
	./src_x/KeypointItem.cpp
	./src_x/QtOpenCV.cpp
	./src_x/Camera.cpp
	./src_x/CameraTcpServer.cpp
	./src/CameraROS.cpp
	./src/FindObjectROS.cpp
	./src_x/ParametersToolBox.cpp
	./src_x/Settings.cpp
	./src_x/FindObject.cpp
	./src_x/ObjWidget.cpp
	./src_x/TcpServer.cpp
	./src_x/Vocabulary.cpp
	./src_x/RectItem.cpp
	./src_x/AboutDialog.cpp
	./src_x/JsonWriter.cpp
	./src_x/utilite/UPlot.cpp
	./src_x/utilite/UFile.cpp
	./src_x/utilite/UDirectory.cpp
	./src_x/utilite/UConversion.cpp
	./src_x/utilite/ULogger.cpp
	./src_x/rtabmap/PdfPlot.cpp
	./src_x/ImageDropWidget.cpp
	./src_x/json/jsoncpp.cpp
	${moc_srcs} 
	${moc_uis} 
	${srcs_qrc}
)

SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
	${CMAKE_CURRENT_SOURCE_DIR}/src_x
	${CMAKE_CURRENT_SOURCE_DIR}/include_x
	${CMAKE_CURRENT_BINARY_DIR} # for qt ui generated in binary dir
	${catkin_INCLUDE_DIRS}
)

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

SET(LIBRARIES
    ${QT_LIBRARIES} 
    ${catkin_LIBRARIES}
    opencv_nonfree
)

#include files
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

add_executable(find_object_2d src/find_object_2d_node.cpp ${SRC_FILES})
target_link_libraries(find_object_2d ${LIBRARIES})
add_dependencies(find_object_2d find_object_2d_generate_messages_cpp)

add_executable(print_objects_detected src/print_objects_detected_node.cpp)
target_link_libraries(print_objects_detected ${LIBRARIES})
add_dependencies(print_objects_detected find_object_2d_generate_messages_cpp)

add_executable(tf_example src/tf_example_node.cpp)
target_link_libraries(tf_example ${LIBRARIES})
add_dependencies(tf_example find_object_2d_generate_messages_cpp)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS 
   find_object_2d 
   print_objects_detected 
   tf_example 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   launch/find_object_2d_gui.launch
   launch/find_object_2d.launch
   launch/find_object_3d.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
