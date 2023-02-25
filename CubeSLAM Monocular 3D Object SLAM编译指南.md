# Cubeslam调试

运行环境：ubuntu20

pangolin=0.5

opencv3=3.4.16

opencv4=4.6.0



**前言**：由于该开源使用的cv库版本较早，在ubuntu20与新的cv库上运行有非常非常非常非常非常多的bug

根据开源部分 make时报错

```cmake
CMake Error at /opt/ros/noetic/share/catkin/cmake/empy.cmake:30 (message):
  Unable to find either executable 'empy' or Python module 'em'...  try
  installing the package 'python3-empy'

```



修改catkinmake指令为

```cmake
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

使用本地python3编译

即

```cmake
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```



这时报错

```
error: ‘std::vector’ has not been declared
```





opencv使用版本为2/3 过于老旧放弃



cubeslam_ws/src/cube_slam/detect_3d_cuboid/include/detect_3d_cuboid/matrix_utils.h报错

```cmake
/home/bopang/Object_level_slam/cubeslam/cubeslam_ws/src/cube_slam/detect_3d_cuboid/include/detect_3d_cuboid/matrix_utils.h:47:101: error: ‘std::vector’ has not been declared
 bool read_obj_detection_txt(const std::string txt_file_name, Eigen::MatrixXd &read_number_mat, std::vector<std::string> &strings);
                              
```

解决：

在matrix_utils.h中加

```c++
#include <opencv2/opencv.hpp>

namespace cv
{
    using std::vector;
}
```



cubeslam_ws/src/cube_slam/line_lbd/libs/lsd.cpp报错

```cmake
/home/bopang/Object_level_slam/cubeslam/cubeslam_ws/src/cube_slam/line_lbd/libs/lsd.cpp:1168:32: error: ‘CV_BGR2GRAY’ was not declared in this scope
         cvtColor(_image, gray, CV_BGR2GRAY);
```



解决：

CV_BGR2GRAY改称COLOR_BGR2GRAY



报错

cubeslam_ws/src/cube_slam/detect_3d_cuboid/src/object_3d_util.cpp

cubeslam_ws/src/cube_slam/detect_3d_cuboid/src/box_proposal_detail.cpp

```cmake
error: ‘iota’ was not declared in this scope
```



解决：

加头文件

```c++
#include <numeric>
```



报错：

/usr/include/pcl-1.10/pcl/point_types.h

```cmake
/usr/include/pcl-1.10/pcl/point_types.h:698:1: error: ‘plus’ is not a member of ‘pcl::traits’
 POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointDEM,
```

？？？？？我PCL库怎么报错了？？？？？

解决

在cmakelists中加

```cmake
set(CMAKE_CXX_STANDARD 14)
```



报错：

src/cube_slam/orb_object_slam/src/LoopClosing.cc

src/cube_slam/orb_object_slam/src/System.cc

src/cube_slam/orb_object_slam/src/Viewer.cc

```
/home/bopang/Object_level_slam/cubeslam/cubeslam_ws/src/cube_slam/orb_object_slam/src/LoopClosing.cc:415:13: error: ‘usleep’ was not declared in this scope
             usleep(5000);
```

解决：

加头文件

```c+=
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
```



报错：

src/cube_slam/object_slam/src/main_obj.cpp

```cmake
/home/bopang/Object_level_slam/cubeslam/cubeslam_ws/src/cube_slam/object_slam/src/main_obj.cpp:311:62: error: ‘CV_LOAD_IMAGE_ANYDEPTH’ was not declared in this scope
       cv::Mat raw_depth_img = cv::imread(raw_depth_img_name, CV_LOAD_IMAGE_ANYDEPTH);
```

解决：

使用cv::IMREAD_ANYDEPTH替换全部CV_LOAD_IMAGE_ANYDEPTH



报错：

src/cube_slam/orb_object_slam/src/Frame.cc

```cmake
/home/bopang/Object_level_slam/cubeslam/cubeslam_ws/src/cube_slam/orb_object_slam/src/Frame.cc:226:54: error: ‘CV_LOAD_IMAGE_UNCHANGED’ was not declared in this scope
         objmask_img = cv::imread(pred_mask_img_name, CV_LOAD_IMAGE_UNCHANGED); // uint8  sometimes read image might take long time....
```

解决

使用cv::IMREAD_UNCHANGED替换全部CV_LOAD_IMAGE_UNCHANGED



报错：

src/cube_slam/orb_object_slam/src/PnPsolver.cc

```
/home/bopang/Object_level_slam/cubeslam/cubeslam_ws/src/cube_slam/orb_object_slam/src/PnPsolver.cc:424:26: error: ‘CV_SVD’ was not declared in this scope
   cvInvert(&CC, &CC_inv, CV_SVD);
```

解决：

加两个头文件

```c++
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
```



报错：

src/cube_slam/orb_object_slam/include/PnPsolver.h

```cmake
/home/bopang/Object_level_slam/cubeslam/cubeslam_ws/src/cube_slam/orb_object_slam/include/PnPsolver.h:98:15: error: ‘CvMat’ has not been declared
   void fill_M(CvMat * M, const int row, const double * alphas, const double u, const double v);
```

解决：

```c++
#include <opencv2/core/types_c.h>
```



报错：

src/cube_slam/orb_object_slam/src/Sim3Solver.cc

```c++
/home/bopang/Object_level_slam/cubeslam/cubeslam_ws/src/cube_slam/orb_object_slam/src/Sim3Solver.cc:215:25: error: ‘CV_REDUCE_SUM’ was not declared in this scope
     cv::reduce(P, C, 1, CV_REDUCE_SUM);
                         ^~~~~~~~~~~~~
```

解决：

```c+=
#include<opencv2/core/core_c.h>
```

至此顺利编译

### 

































