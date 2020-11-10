# Lidar_Stereo_Calibration
This is our work based on the submitted paper Lidar stereo calibration using correntropy criterion.
We have added sample data sets to test the work with particular scenarios. The test cases are present in the build directory.


###### Example test cases are : gz_test1_li.pcd, gz_test1_st.pcd ,robot_test1_li.pcd ,robot_test1_st.pcd etc.,

NOTE: Please edit the CMakeLists.txt file to change the location of the PCL based on your system configuration.

Open the terminal and  navigate to the build diretory .
 In the terminal, type the following :
 
 > cmake ..

 > make 
 
 Before running the algorithm please the following parameters for getting better results:

* In function detectKeypoints_sift3d() in line 89 and 90 , change the last 4 input paramters for getting the required number of input points using sift algorithm. The last 4 parameters realtes to (int min_scale,int n_octaves,int n_scales_per_octave, double min_contrast), which are the parameters for sift algorithm

 * In line 328 and 329 change corr_th(between 0 to 1 e.g., 0.98) and ang_th (e.g., 0.25) to get 
 
 
 Run the algorithm using the command.
 
> ./test_6 gz_test1_st.pcd gz_test1_li.pcd

The first input parameter corresponds to Stereo point cloud data and the second input parameter corresponds to Lidar point cloud  data configuration.

The results are in the form of a transformation matrix .


```
  transformation_matrix=
   0.998232  -0.0178253   0.0567091   0.0687287
  0.0175786    0.999834  0.00484514   -0.117551
 -0.0567864 -0.00383972     0.99838    0.139574
          0           0           0           1
OK

```
