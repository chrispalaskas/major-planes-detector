# major-planes-detector #
- Detects the n major planes of a 3D point cloud using the PCL library
    - (https://github.com/PointCloudLibrary/pcl/releases)
    - PCL-1.9.1-AllInOne-msvc2017-win64.exe<br /> prebuilt works fine for Visual Studio 2019 Preview
    - Make sure to add the path to the PCL and OpenNI2 dlls to the env path
        - e.g C:/Program Files/PCL 1.9.1/bin/ and C:/Program Files/OpenNI2/Redist/
- Unit testing is powered by Google Unit Tests. (https://github.com/google/googletest/)
    - When cloning project from GIT also Upload Module googletest
    - When running CMake use gtest_force_shared_crt for googletest
- Documentation can be provided by running doxygen on the source files.

## Usage ##

