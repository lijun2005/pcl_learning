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

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE , 2, "demo_cloud");
  while( !viewer->wasStopped())
  {
    viewer->spinOnce(); //每次循环调用内部的重绘函数，按 q 退出
  }

  return 0;
}
```
### 读入数据，随机创建初始点，采用半径范围搜索和k近邻搜索
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
#include <pcl/kdtree/kdtree_flann.h>

int main(int argc, char **argv) {
  srand(time(NULL));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr  k_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  k_cloud->width = 1000;
  k_cloud->height = 1; //无序点云
  k_cloud->points.resize(k_cloud->width * k_cloud->height);
  pcl::PointCloud<pcl::PointXYZ>::Ptr r_cloud(new pcl::PointCloud<pcl::PointXYZ>( *cloud));
  r_cloud->width = 1000;
  r_cloud->height = 1; //无序点云
  r_cloud->points.resize(k_cloud->width * k_cloud->height);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  if( pcl::io::loadPCDFile("/home/lijun/pcl/src/pcl_learning/src/demo.pcd", *cloud) < 0) 
  {
    std::cerr<< " no this .pcd file !" << std::endl;
    return -1;
  }
  kdtree.setInputCloud( cloud);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> r_viewer(new pcl::visualization::PCLVisualizer());
  boost::shared_ptr<pcl::visualization::PCLVisualizer> k_viewer(new pcl::visualization::PCLVisualizer());
  k_viewer->setBackgroundColor(0,0,0);
  r_viewer->setBackgroundColor(0,0,0);

  pcl::PointXYZ  search_point;
  search_point.x = 1024.0* rand() / (RAND_MAX + 1.0);
  search_point.y = 1024.0* rand() / (RAND_MAX + 1.0);
  search_point.z = 1024.0* rand() / (RAND_MAX + 1.0);


  // //k near search
  int k=10;
  std::vector<int> pointsidx_k(k);
  std::vector<float> distance_k(k);
  if(kdtree.nearestKSearch(search_point , k, pointsidx_k , distance_k) > 0 )
  {
    for(int i=0 ; i < pointsidx_k.size(); i++)
    {
      k_cloud->points[i].x = cloud->points[pointsidx_k[i]].x;
      k_cloud->points[i].y = cloud->points[pointsidx_k[i]].y;
      k_cloud->points[i].z = cloud->points[pointsidx_k[i]].z;
    }
  std::cout << "K nearest neighbor search at (" << search_point.x 
          << " " << search_point.y 
          << " " << search_point.z
          << ") with K=" << k << std::endl;
  for (int i = 0; i < pointsidx_k.size (); ++i)
    std::cout << "    "  <<   cloud->points[pointsidx_k[i]].x 
              << " " << cloud->points[pointsidx_k[i]].y
              << " " << cloud->points[pointsidx_k[i]].z
              << " (squared distance: " << distance_k[i] << ")" << std::endl;
    //printf("success");

  }
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_k_cloud (k_cloud , 255,255,255);

  k_viewer->addPointCloud<pcl::PointXYZ> (k_cloud , color_k_cloud , "k_cloud");
  k_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE , 2 , "k_cloud");


  //radius search
  float radius = 256.0 * rand()/(RAND_MAX + 1.0);
  std::vector<int> pointsidx_r;
  std::vector<float> distance_r;

  if(kdtree.radiusSearch(search_point , radius, pointsidx_r , distance_r) > 0 )
  {
    for(int i=0 ; i < pointsidx_r.size(); i++)
    {
      r_cloud->points[pointsidx_r[i]].x = cloud->points[pointsidx_r[i]].x;
      r_cloud->points[pointsidx_r[i]].y = cloud->points[pointsidx_r[i]].y;
      r_cloud->points[pointsidx_r[i]].z = cloud->points[pointsidx_r[i]].z;
    }
    std::cout << "K nearest neighbor search at (" << search_point.x 
          << " " << search_point.y 
          << " " << search_point.z
          << ") with K=" << k << std::endl;
    for (int i = 0; i < pointsidx_r.size (); ++i)
    std::cout << "    "  <<   cloud->points[pointsidx_r[i]].x 
              << " " << cloud->points[pointsidx_r[i]].y
              << " " << cloud->points[pointsidx_r[i]].z
              << " (squared distance: " << distance_r[i] << ")" << std::endl;
  }
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_r_cloud (r_cloud , 255,255,255);
  r_viewer->addPointCloud<pcl::PointXYZ> (r_cloud , color_r_cloud , "r_cloud");
  r_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE , 2 , "r_cloud");

  while(!k_viewer->wasStopped() && !r_viewer->wasStopped())
  {
    k_viewer->spinOnce();
    r_viewer->spinOnce();
  }

  return 0;
}
```

