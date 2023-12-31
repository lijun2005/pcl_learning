## 任务介绍
- 了解octree的空间基本理论
- 学习如何压缩单点云和点云流
- 学习如何使用八叉树进行空间分区和最近邻搜索
- 学习如何使用八叉树检测点云内的空间变化
- 知道octree和kdtree的区别以及不同使用场景
- 学会利用pcl_octree_viewer工具来看cotree

## octree与kdtree
在PCL (Point Cloud Library) 中，Octree（八叉树）和KdTree（K维树）都是用于处理点云数据的数据结构，但它们有不同的特点和用途。
### Octree（八叉树）：
Octree 是一种基于递归的树状数据结构，用于对三维空间进行划分。每个节点都代表一个边界框（Bounding Box），并且树的每个层级都将空间划分为八个子区域，因此称为八叉树。这样的划分允许在不同层级上对点云数据进行粗略或细致的处理。一般来说，较近的点更有可能在同一个节点内，这使得八叉树在某些应用中具有优势。

用途：

- 空间分区：八叉树适用于空间分区任务，特别是需要处理大规模点云数据时。它可以帮助加速搜索和查询特定区域的点。
- 点云压缩：通过将一些节点合并，八叉树可以用于点云的压缩和减少存储空间。
- 点云采样：八叉树可以用于点云采样，从原始数据中提取具有代表性的点子集。


Octree（八叉树）的应用场景：

    空间分区和可视化：
        场景：处理大规模点云数据时，Octree可以将三维空间划分为具有不同层级的节点。这种分层结构允许更高效地组织和管理点云数据，使得可视化和渲染变得更加简单和高效。
        例子：在虚拟现实环境中，对大规模点云数据进行渲染和展示。

    点云压缩：
        场景：大规模点云数据可能会占用大量的存储空间，Octree可以用于压缩点云数据。
        例子：在云数据存储和传输方面，使用Octree来减少存储和传输所需的带宽和存储空间。

    点云采样：
        场景：在某些应用中，需要从原始点云数据中提取具有代表性的子集。
        例子：在点云配准和目标检测中，通过Octree进行点云采样，可以提高算法的效率和鲁棒性。

    碰撞检测：
        场景：在虚拟环境、游戏开发或机器人运动规划等领域，需要进行碰撞检测。
        例子：在机器人路径规划中，使用Octree来检测机器人与环境中物体之间的碰撞，以避免碰撞。
  
#### KdTree（K维树）：
KdTree 是一种基于二叉树的数据结构，用于对K维空间中的点进行划分。它是一种空间划分树，通过选择维度中的中值来构建树的节点。每个节点代表一个超矩形边界框（Hyper-Rectangle），并根据点在每个维度上的值将空间划分为两个子区域。KdTree 在低维空间中表现良好，但在高维空间中可能效率降低。

用途：

- 最近邻搜索：KdTree 可以高效地执行最近邻搜索，找到给定点在点云中距离最近的点。
- 点云配准：在点云配准任务中，KdTree 可以用于查找匹配点对，从而实现点云的对齐。
- 特征提取：通过在KdTree上执行邻域搜索，可以用于点云特征提取，如法向量估计和曲率计算

KdTree（K维树）的应用场景：

    最近邻搜索：
        场景：在许多点云应用中，需要找到给定点云中距离某个查询点最近的点。这可以用于确定点云中每个点的最近邻点，或者在点云数据中查找最接近特定位置的点。
        例子：机器人导航中的碰撞检测，需要查找距离机器人当前位置最近的障碍物点。

    点云配准：
        场景：在点云配准过程中，需要找到两个或多个点云之间的匹配点对，以实现点云的对齐。
        例子：在三维重建中，需要将多个扫描或视角的点云对齐，形成完整的三维模型。

    特征提取：
        场景：对于点云的特征提取，例如计算法向量、曲率、表面描述符等，需要在每个点的邻域内进行分析。
        例子：在目标识别中，需要计算点云表面的法向量来判断物体的朝向。


### 综合应用场景：

有些情况下，KdTree和Octree也可以结合使用。例如：

    在对大规模点云数据进行最近邻搜索时，可以首先使用Octree对点云进行空间划分，然后在每个八叉树节点上构建KdTree，以实现更高效的最近邻搜索。
    在点云配准任务中，可以使用KdTree来进行初步的最近邻匹配，然后使用Octree来加速迭代的配准过程。
### 用无人机举例
#### KdTree的应用场景：

    无人机航线规划：
        场景：在无人机航线规划中，需要规划无人机的飞行路径以收集点云数据。最近邻搜索是其中一个关键步骤，用于确定无人机当前位置附近的最佳航点，以避免障碍物或选择最优的航迹点。

    地面建模与检测：
        场景：无人机在航行中通过激光雷达或RGB-D摄像头采集地面数据，用于地形建模和检测地面上的障碍物或特定目标。
        例子：使用KdTree进行最近邻搜索，以找到地面点和非地面点之间的边界，从而区分地面和障碍物。

    点云过滤和降噪：
        场景：无人机从传感器中获取的原始点云数据可能包含噪声或无用信息。在进行后续处理之前，需要对点云进行过滤和降噪。
        例子：使用KdTree进行最近邻搜索，从原始点云中去除离群点或过滤掉不需要的数据。

#### Octree的应用场景：

    点云压缩：
        场景：无人机在飞行过程中可能会生成大量的点云数据，这些数据需要进行存储和传输。对于大规模数据集，点云压缩是至关重要的。
        例子：使用Octree来对点云数据进行空间划分，然后根据压缩需求合并和采样节点，从而减少点云数据的存储和传输量。

    目标检测与识别：
        场景：无人机携带传感器采集点云数据，用于目标检测和识别，例如检测建筑物、车辆或植被等。
        例子：通过Octree将点云数据进行空间划分，并在每个节点上提取特征，用于目标的识别和分类。

    场景可视化和实时处理：
        场景：无人机采集的点云数据可能是动态的或实时更新的。在展示和可视化场景时，需要高效地处理和渲染点云数据。
        例子：利用Octree的分层结构，可以高效地管理大规模的点云数据，实现实时的场景可视化和交互。

#### 综合应用场景：

    在无人机航线规划过程中，可以使用KdTree进行路径规划和障碍物避障，同时利用Octree来对无人机获取的点云数据进行空间分区和压缩，以便在实时展示和后续处理中提高效率
 ## pcl_octree_viewer工具使用
- a->增加显示深度（减小体素大小）
- z->降低显示深度（增加体素大小）
- v->隐藏或显示octree立方体
- b->隐藏或显示中心点
- n->隐藏或显示原始点云
- q->退出

## 八叉树的空间划分和三种搜索方法
### 代码实现
```c++
#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc , char **argv)
{
  srand(time(NULL));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr v_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  v_cloud->width = 1000;
  v_cloud ->height = 1;
  v_cloud->resize(1000*1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr r_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  r_cloud->width = 1000;
  r_cloud ->height = 1;
  r_cloud->resize(1000*1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr k_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  k_cloud->width = 1000;
  k_cloud ->height = 1;
  k_cloud->resize(1000*1);
  pcl::io::loadPCDFile("/home/lijun/pcl/src/pcl_learning/src/demo.pcd", *cloud);

  pcl::PointXYZ search_point;
  search_point.x = 1024.0 *rand()/(RAND_MAX + 1.0);
  search_point.y = 1024.0 *rand()/(RAND_MAX + 1.0);
  search_point.z = 1024.0 *rand()/(RAND_MAX + 1.0);

  //全局共享智能指针
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D viewer"));
  viewer->setBackgroundColor(0,0,0);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> v_viewer (new pcl::visualization::PCLVisualizer("3D viewer"));
  v_viewer->setBackgroundColor(0,0,0);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> r_viewer (new pcl::visualization::PCLVisualizer("3D viewer"));
  r_viewer->setBackgroundColor(0,0,0);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> k_viewer (new pcl::visualization::PCLVisualizer("3D viewer"));
  k_viewer->setBackgroundColor(0,0,0);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloud( cloud,0,255,0 );
  viewer->addPointCloud(cloud , color_cloud , "raw cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE , 2 , "raw cloud");

  float resolution = 128.0; //单位体素大小
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  //voxel search
  std::vector<int> v_pointidx;
  if(octree.voxelSearch(search_point , v_pointidx) > 0)
  {
    for(int i=0 ; i<v_pointidx.size(); i++)
    {
      v_cloud->points[i].x = cloud->points[v_pointidx[i]].x;
      v_cloud->points[i].y = cloud->points[v_pointidx[i]].y;
      v_cloud->points[i].z = cloud->points[v_pointidx[i]].z;
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> v_color_cloud(v_cloud , 255,0,0);
    v_viewer->addPointCloud(v_cloud , v_color_cloud, "v cloud");
    v_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE , 1 , "v cloud");
        for (int i = 0; i < v_pointidx.size(); ++i)
            std::cout << "    " << cloud->points[v_pointidx[i]].x
                      << " " << cloud->points[v_pointidx[i]].y
                      << " " << cloud->points[v_pointidx[i]].z
                      <<  std::endl;
  }


  //radius search
  float radius = 256.0 *rand() / (RAND_MAX + 1.0);
  std::vector<int> r_pointidx;
  std::vector<float> r_distance;

  if(octree.radiusSearch(search_point , radius , r_pointidx , r_distance) > 0)
  {
    for(int i=0 ; i<r_pointidx.size(); i++)
    {
      r_cloud->points[i].x = cloud->points[r_pointidx[i]].x;
      r_cloud->points[i].y = cloud->points[r_pointidx[i]].y;
      r_cloud->points[i].z = cloud->points[r_pointidx[i]].z;
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> r_color_cloud(r_cloud , 0,0,255);
    r_viewer->addPointCloud(r_cloud , r_color_cloud, "r cloud");
    r_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE , 1 , "r cloud");
    for (int i = 0; i < r_pointidx.size(); ++i)
            std::cout << "    " << cloud->points[r_pointidx[i]].x
                      << " " << cloud->points[r_pointidx[i]].y
                      << " " << cloud->points[r_pointidx[i]].z
                      << " (squared distance: " << r_distance[i] << ")" << std::endl;
  }

  //k search
  int k = 10;
  std::vector<int> k_pointidx;
  std::vector<float> k_distance;

  if(octree.nearestKSearch(search_point , k , k_pointidx , k_distance))
  {
    for(int i=0 ; i<k_pointidx.size(); i++)
    {
      k_cloud->points[i].x = cloud->points[k_pointidx[i]].x;
      k_cloud->points[i].y = cloud->points[k_pointidx[i]].y;
      k_cloud->points[i].z = cloud->points[k_pointidx[i]].z;
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> k_color_cloud(k_cloud , 255,255,255);
    k_viewer->addPointCloud(k_cloud , k_color_cloud, "k cloud");
    k_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE , 1 , "k cloud");
    for (int i = 0; i < k_pointidx.size(); ++i)
            std::cout << "    " << cloud->points[k_pointidx[i]].x
                      << " " << cloud->points[k_pointidx[i]].y
                      << " " << cloud->points[k_pointidx[i]].z
                      << " (squared distance: " << k_distance[i] << ")" << std::endl;
  }

  viewer->addCoordinateSystem(0.5);
  v_viewer->addCoordinateSystem(0.5);
  r_viewer->addCoordinateSystem(0.5);
  k_viewer->addCoordinateSystem(0.5);
  while(!viewer->wasStopped())
  {
    viewer->spinOnce();
    r_viewer->spinOnce();
    k_viewer->spinOnce();
    v_viewer->spinOnce();
  }
  return 0;
}
```
### 结果与kdtree搜索算法的结果对比分析
