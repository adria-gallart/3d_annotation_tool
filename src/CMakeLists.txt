project(Annotation_tool_exe)


find_package(PCL 1.4 REQUIRED COMPONENTS visualization filters segmentation)
find_package(Qt4 4.7 REQUIRED)
FIND_PACKAGE(VTK REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${QT_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS})

include(${QT_USE_FILE})

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ./../../bin/)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

link_directories(${PCL_LIBRARY_DIRS} ${QT_LIBRARY_DIRS} ${VTK_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${QT_DEFINITIONS})

SET(app_tool_SOURCES main.cpp mainwindow.cpp pointcloudmodifier.cpp viewerinteractor.cpp newobjectdialog.cpp objectsinformation.cpp selectobjectdialog.cpp)
SET(app_tool_HEADERS mainwindow.h pointcloudmodifier.h viewerinteractor.h newobjectdialog.h objectsinformation.h selectobjectdialog.h)
SET(app_tool_UIS newobjectdialog.ui mainwindow.ui selectobjectdialog.ui)
SET(QtApp_RCCS ./../icons/icons.qrc)

# this will run uic on .ui files:
QT4_WRAP_UI(app_tool_HDRS ${app_tool_UIS} )

# generate rules for building source files from the resources
QT4_ADD_RESOURCES(QtApp_RCC_SRCS ${QtApp_RCCS})

# generate rules for building source files that moc generates
QT4_WRAP_CPP(app_tool_HEADERS_MOC ${app_tool_HEADERS})

add_executable (Annotation_tool ${app_tool_SOURCES} ${app_tool_HEADERS_MOC} ${QtApp_RCC_SRCS} ${app_tool_HDRS})
target_link_libraries (Annotation_tool QVTK vtkHybrid ${PCL_LIBRARIES} ${QT_LIBRARIES} pcl_visualization) 
