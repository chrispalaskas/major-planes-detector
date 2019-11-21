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
- The executable can be called with up to 4 arguments:
    - (Mandatory): Input File Path: A file containing the point cloud. Acceptable types:
        - Simple text file of the format {x y z}, one point per line delimited with spaces, not commas and with the extension .xyz
        - PCL cloud point file format, either ASCII or encoded with the extension .pcd.
    - (Mandatory): Number of planes to detect: An integer. If the points are exhausted before the planes reach this number the rest will be omitted.
    - (Optional): Distance threshold: A double that is used to parameterize the segmentation algorithm. If set too small, all points will be considered in the same plane. It should be set depending on the how close to each other are the points. The smaller the scale the smaller the threshold. Default value is 1.0.
    - (Optional): Enable Visualization: A boolean that enables visualization using the PCLVisualizer. When set to true an interactive window appears showing the whole point cloud in 3D space, while the rest of the processing halts. Once the window is terminated, another appears showing the point cloud of the major plane and a vector from the centroid of the plane. This is repeated for all planes detected. Finally a window with the remaining (unclassified) points appears.
- Example: pcl_detect_planes.exe ..\cloud3D.xyz 3 0.01 true
