# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/ubuntu/Documents/pcl_demos/6PCL_extract_indices (copy)"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/ubuntu/Documents/pcl_demos/6PCL_extract_indices (copy)/build"

# Include any dependencies generated for this target.
include CMakeFiles/extractIndices.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/extractIndices.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/extractIndices.dir/flags.make

CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o: CMakeFiles/extractIndices.dir/flags.make
CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o: ../src/PCL_extract_indices.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/ubuntu/Documents/pcl_demos/6PCL_extract_indices (copy)/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o -c "/home/ubuntu/Documents/pcl_demos/6PCL_extract_indices (copy)/src/PCL_extract_indices.cpp"

CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/ubuntu/Documents/pcl_demos/6PCL_extract_indices (copy)/src/PCL_extract_indices.cpp" > CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.i

CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/ubuntu/Documents/pcl_demos/6PCL_extract_indices (copy)/src/PCL_extract_indices.cpp" -o CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.s

CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o.requires:

.PHONY : CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o.requires

CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o.provides: CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o.requires
	$(MAKE) -f CMakeFiles/extractIndices.dir/build.make CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o.provides.build
.PHONY : CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o.provides

CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o.provides.build: CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o


# Object files for target extractIndices
extractIndices_OBJECTS = \
"CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o"

# External object files for target extractIndices
extractIndices_EXTERNAL_OBJECTS =

extractIndices: CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o
extractIndices: CMakeFiles/extractIndices.dir/build.make
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_system.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_thread.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_regex.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpthread.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_common.so
extractIndices: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_search.so
extractIndices: /usr/lib/libOpenNI.so
extractIndices: /usr/lib/x86_64-linux-gnu/libz.so
extractIndices: /usr/lib/x86_64-linux-gnu/libjpeg.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpng.so
extractIndices: /usr/lib/x86_64-linux-gnu/libtiff.so
extractIndices: /usr/lib/x86_64-linux-gnu/libfreetype.so
extractIndices: /usr/lib/x86_64-linux-gnu/libnetcdf_c++.so
extractIndices: /usr/lib/x86_64-linux-gnu/libnetcdf.so
extractIndices: /usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5.so
extractIndices: /usr/lib/x86_64-linux-gnu/libsz.so
extractIndices: /usr/lib/x86_64-linux-gnu/libdl.so
extractIndices: /usr/lib/x86_64-linux-gnu/libm.so
extractIndices: /usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5_hl.so
extractIndices: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
extractIndices: /usr/lib/x86_64-linux-gnu/libexpat.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpython2.7.so
extractIndices: /usr/lib/libgl2ps.so
extractIndices: /usr/lib/x86_64-linux-gnu/libtheoraenc.so
extractIndices: /usr/lib/x86_64-linux-gnu/libtheoradec.so
extractIndices: /usr/lib/x86_64-linux-gnu/libogg.so
extractIndices: /usr/lib/x86_64-linux-gnu/libxml2.so
extractIndices: /usr/lib/libvtkWrappingTools-6.2.a
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_io.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_features.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
extractIndices: /usr/lib/x86_64-linux-gnu/libqhull.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_people.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_system.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_thread.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
extractIndices: /usr/lib/x86_64-linux-gnu/libboost_regex.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpthread.so
extractIndices: /usr/lib/x86_64-linux-gnu/libqhull.so
extractIndices: /usr/lib/libOpenNI.so
extractIndices: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkImagingStencil-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libz.so
extractIndices: /usr/lib/x86_64-linux-gnu/libjpeg.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpng.so
extractIndices: /usr/lib/x86_64-linux-gnu/libtiff.so
extractIndices: /usr/lib/x86_64-linux-gnu/libfreetype.so
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOParallelNetCDF-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libnetcdf_c++.so
extractIndices: /usr/lib/x86_64-linux-gnu/libnetcdf.so
extractIndices: /usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpthread.so
extractIndices: /usr/lib/x86_64-linux-gnu/libsz.so
extractIndices: /usr/lib/x86_64-linux-gnu/libdl.so
extractIndices: /usr/lib/x86_64-linux-gnu/libm.so
extractIndices: /usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5_hl.so
extractIndices: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
extractIndices: /usr/lib/x86_64-linux-gnu/libexpat.so
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkLocalExample-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libpython2.7.so
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkTestingGenericBridge-6.2.so.6.2.0
extractIndices: /usr/lib/libgl2ps.so
extractIndices: /usr/lib/x86_64-linux-gnu/libtheoraenc.so
extractIndices: /usr/lib/x86_64-linux-gnu/libtheoradec.so
extractIndices: /usr/lib/x86_64-linux-gnu/libogg.so
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOMINC-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingImage-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libxml2.so
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersReebGraph-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOXdmf2-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOAMR-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkImagingStatistics-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOParallel-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIONetCDF-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtOpenGL-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOParallelLSDyna-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelGeometry-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtWebkit-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.2.so.6.2.0
extractIndices: /usr/lib/libvtkWrappingTools-6.2.a
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersHyperTree-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeOpenGL-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOPostgreSQL-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkWrappingJava-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelFlowPaths-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelStatistics-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersProgrammable-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelImaging-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingParallelLIC-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingLIC-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersPython-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOParallelExodus-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneric-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOVideo-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingQt-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersTexture-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOInfovis-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtSQL-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeOpenGL-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkInfovisBoostGraphAlgorithms-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOGeoJSON-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersVerdict-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkViewsGeovis-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOImport-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkTestingIOSQL-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOODBC-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOEnSight-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOMySQL-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingMatplotlib-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkDomainsChemistry-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOExport-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelMPI-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOParallelXML-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkTestingRendering-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOMPIParallel-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkParallelMPI4Py-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersSMP-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersSelection-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOVPIC-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkVPIC-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkImagingMath-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkImagingMorphological-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingParallel-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeFontConfig-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOFFMPEG-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOMPIImage-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOGDAL-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_common.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_search.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_io.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_features.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_people.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
extractIndices: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkxdmf2-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libxml2.so
extractIndices: /usr/lib/x86_64-linux-gnu/hdf5/openmpi/lib/libhdf5.so
extractIndices: /usr/lib/x86_64-linux-gnu/hdf5/openmpi/lib/libhdf5_hl.so
extractIndices: /usr/lib/x86_64-linux-gnu/hdf5/openmpi/lib/libhdf5.so
extractIndices: /usr/lib/x86_64-linux-gnu/hdf5/openmpi/lib/libhdf5_hl.so
extractIndices: /usr/lib/x86_64-linux-gnu/libsz.so
extractIndices: /usr/lib/x86_64-linux-gnu/libdl.so
extractIndices: /usr/lib/x86_64-linux-gnu/libm.so
extractIndices: /usr/lib/x86_64-linux-gnu/libsz.so
extractIndices: /usr/lib/x86_64-linux-gnu/libdl.so
extractIndices: /usr/lib/x86_64-linux-gnu/libm.so
extractIndices: /usr/lib/openmpi/lib/libmpi.so
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOLSDyna-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkViewsQt-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersAMR-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersFlowPaths-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOExodus-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkexoIIc-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libnetcdf_c++.so
extractIndices: /usr/lib/x86_64-linux-gnu/libnetcdf.so
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.5.1
extractIndices: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.5.1
extractIndices: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.5.1
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkverdict-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkGeovisCore-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkproj4-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkViewsInfovis-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkInfovisLayout-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersImaging-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOSQL-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkWrappingPython27Core-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkPythonInterpreter-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingLabel-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingGL2PS-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOXML-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libpython2.7.so
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libGLU.so
extractIndices: /usr/lib/x86_64-linux-gnu/libSM.so
extractIndices: /usr/lib/x86_64-linux-gnu/libICE.so
extractIndices: /usr/lib/x86_64-linux-gnu/libX11.so
extractIndices: /usr/lib/x86_64-linux-gnu/libXext.so
extractIndices: /usr/lib/x86_64-linux-gnu/libXt.so
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallel-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkalglib-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkftgl-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libfreetype.so
extractIndices: /usr/lib/x86_64-linux-gnu/libGL.so
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOMovie-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libtheoraenc.so
extractIndices: /usr/lib/x86_64-linux-gnu/libtheoradec.so
extractIndices: /usr/lib/x86_64-linux-gnu/libogg.so
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkParallelMPI-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkParallelCore-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOImage-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkIOCore-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtksys-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libvtkmetaio-6.2.so.6.2.0
extractIndices: /usr/lib/x86_64-linux-gnu/libz.so
extractIndices: CMakeFiles/extractIndices.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/ubuntu/Documents/pcl_demos/6PCL_extract_indices (copy)/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable extractIndices"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/extractIndices.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/extractIndices.dir/build: extractIndices

.PHONY : CMakeFiles/extractIndices.dir/build

CMakeFiles/extractIndices.dir/requires: CMakeFiles/extractIndices.dir/src/PCL_extract_indices.cpp.o.requires

.PHONY : CMakeFiles/extractIndices.dir/requires

CMakeFiles/extractIndices.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/extractIndices.dir/cmake_clean.cmake
.PHONY : CMakeFiles/extractIndices.dir/clean

CMakeFiles/extractIndices.dir/depend:
	cd "/home/ubuntu/Documents/pcl_demos/6PCL_extract_indices (copy)/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/ubuntu/Documents/pcl_demos/6PCL_extract_indices (copy)" "/home/ubuntu/Documents/pcl_demos/6PCL_extract_indices (copy)" "/home/ubuntu/Documents/pcl_demos/6PCL_extract_indices (copy)/build" "/home/ubuntu/Documents/pcl_demos/6PCL_extract_indices (copy)/build" "/home/ubuntu/Documents/pcl_demos/6PCL_extract_indices (copy)/build/CMakeFiles/extractIndices.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/extractIndices.dir/depend

