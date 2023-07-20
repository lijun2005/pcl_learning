## 任务
- 了解kdtree的基本知识
- 利用基本的输入输出以及可视化的知识创建随机点构建点云图像并保存
- 读入自己创建的点云图像，利用kdtree的相关接口k近邻和半径搜索出得到的点并将这些点保存同时利用可视化将其显示
## 任务代码
### 创建点云并保存
```c++
#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char **argv) {
  srand(time(NULL));//构建时间初始化种子
  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 1000;
  cloud->height = 1; //无序点云
  cloud->points.resize(cloud->width * cloud->height);

  for(int i=0; i < cloud->width ; i++)
  {
    cloud->points[i].x = 1024.0 * rand()/ (RAND_MAX +1.0);
    cloud->points[i].y = 1024.0 * rand()/ (RAND_MAX +1.0);
    cloud->points[i].z = 1024.0 * rand()/ (RAND_MAX +1.0);
  }

  pcl::io::savePCDFileBinary("/home/lijun/pcl/src/pcl_learning/src/demo.pcd" , *cloud);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ());

  viewer->setBackgroundColor(0,0,0);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloud(cloud, 2555,255,0);
  viewer->addPointCloud<pcl::PointXYZ>  (cloud , color_cloud , " demo_cloud");

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE , 2, "demo_cloud");
  while( !viewer->wasStopped())
  {
    viewer->spinOnce();
  }

  return 0;
}









```

