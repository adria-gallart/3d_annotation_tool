================================
README: Annotation tool
================================
as
:author: Adrià Gallart del Burgo
:e-mail: adriagallart@gmail.com 

Annotation tool is an application designed to annotate objects 
in a point cloud scene. It was developed to annotate 
the objects of a tabletop, specifically a desk.

================================
CONTENTS
================================

The directory contains four folders and three files:

 - bin: folder where is located the executable after compiling the application.
 - documentation: folder that contains the user guide.
 - icons: folder with all the icons used.
 - src: this folder contains the source code.
 - annotation_tool.pro: QtCreator project file.
 - CMakeLists.txt: instructions for the installation.
 - README.txt: readme file.

================================
SYSTEM REQUIREMENTS
================================

To install the annotation tool you need:

1. CMake 2.8.x or later
	sudo apt-get install cmake

2. QtCreator 2.4.1
	sudo apt-get install qtcreator

3. PCL 1.6.0
   	Available from: http://www.pointclouds.org/downloads/

4. VTK 5.8
	sudo apt-get -b source vtk



================================
INSTALLATION
================================

To install the annotation tool:

1. Enter the annotation_tool directory

2. mkdir build

3. cd build

4. cmake ..

5. make

================================
USE
================================

To use the annotation tool:

1. Enter the annotation_tool directory

2. cd bin

3. ./Annotation_tool


================================
DEVELOP IN QTCREATOR
================================

To develop the code and modify the apppearance of the application go
to QtCreator and Open file or Project and open the .pro file.

================================
USE QVTK WIDGET IN QT GUI
================================

Since you have built VTK, it generates a library file called libQVTKWidgetPlugin.so. Once you place a copy of this in [QTinstallDir]/plugins/designer/ directory, the QVTKWidget appears in the Qt Widget panel. 


