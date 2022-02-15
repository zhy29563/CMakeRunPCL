#include <iostream>
#include <iomanip>
#include <vector>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

/*
 * 八叉树是一种用于管理稀疏3D数据的树状数据结构，本例中将学习如何利用八叉树实现用于多个无序点云之间的空间变化检测，这些点云可能在尺
 * 寸、分辨率、密度和点顺序等方面有所差异。通过递归地比较八叉树的树结构，可以鉴定出由八茶树产生的体素组成之间的区别所代表的空间变化，
 * 此外，将解释如何使用PCL八叉树“双缓冲”技术，一边能实时地探测多个点云之间的空间组成差异
 */
int main(int argc, char **argv)
{
    srand((unsigned int)time(NULL));

    // 八叉树分辨率，即体素的大小
    float resolution = 32.0f;

    // 点云A
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
    cloudA->width = 128;
    cloudA->height = 1;
    cloudA->points.resize(cloudA->width * cloudA->height);
    for (auto &point : *cloudA)
    {
        point.x = 64.0f * rand() / (RAND_MAX + 1.0f);
        point.y = 64.0f * rand() / (RAND_MAX + 1.0f);
        point.z = 64.0f * rand() / (RAND_MAX + 1.0f);
    }

    // 点云B
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);
    cloudB->width = 128;
    cloudB->height = 1;
    cloudB->points.resize(cloudB->width * cloudB->height);
    for (auto &point : *cloudB)
    {
        point.x = 64.0f * rand() / (RAND_MAX + 1.0f);
        point.y = 64.0f * rand() / (RAND_MAX + 1.0f);
        point.z = 64.0f * rand() / (RAND_MAX + 1.0f);
    }

    // 初始化空间变换检测对象
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(cloudA);
    octree.addPointsFromInputCloud();
    octree.switchBuffers(); // 交换八叉树缓存，但是cloudA对应的八叉树结构仍在内存中
    octree.setInputCloud(cloudB);
    octree.addPointsFromInputCloud();

    // 为了获取存在于cloudB的点集R（此R中没有cloudA中元素），可以调用getPointIndicesFromNewVoxels方法，通过探测两个八叉树之间体
    // 素的不同来实现。它返回cloudB中新加点的索引向量，通过索引向量可以获取R点集
    std::vector<int> newPointIdxVector;
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);

    // Output points
    std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
    for (std::size_t i = 0; i < newPointIdxVector.size(); ++i)
        std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                  << i << "# Index:" << newPointIdxVector[i]
                  << "  Point:" << (*cloudB)[newPointIdxVector[i]].x << " "
                  << (*cloudB)[newPointIdxVector[i]].y << " "
                  << (*cloudB)[newPointIdxVector[i]].z << std::endl;

    return EXIT_SUCCESS;
}