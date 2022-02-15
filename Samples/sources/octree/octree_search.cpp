#include <iostream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <vector>
#include <ctime>

int main(int argc, char **argv)
{
    /*
     * 八叉树搜索
     *
     * 八叉树是一种用于管理 稀疏 3D数据的树状数据结构，每个内部节点都正好有八个子节点。
     */

    srand(static_cast<unsigned>(time(nullptr)));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 创建点云数据
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }

    const auto resolution = 128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    // 体素内近邻搜索
    std::vector<int> pointIdxVec;
    if (octree.voxelSearch(searchPoint, pointIdxVec))
    {
        std::cout << "Neighbors within voxel search at ("
                  << std::setw(10) << searchPoint.x
                  << std::setw(10) << searchPoint.y
                  << std::setw(10) << searchPoint.z << ")"
                  << std::endl;

        for (size_t i = 0; i < pointIdxVec.size(); ++i)
            std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                      << std::setw(10) << cloud->points[pointIdxVec[i]].x
                      << std::setw(10) << cloud->points[pointIdxVec[i]].y
                      << std::setw(10) << cloud->points[pointIdxVec[i]].z
                      << std::endl;
    }

    // K近邻搜索
    const auto k = 10;
    std::vector<int> point_idx_nkn_search;
    std::vector<float> point_nkn_squared_distance;
    std::cout << "K nearest neighbor search at ("
              << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
              << std::setw(10) << searchPoint.x
              << std::setw(10) << searchPoint.y
              << std::setw(10) << searchPoint.z << ") with K="
              << std::setw(10) << k
              << std::endl;

    if (octree.nearestKSearch(searchPoint, k, point_idx_nkn_search, point_nkn_squared_distance) > 0)
    {
        for (size_t i = 0; i < point_idx_nkn_search.size(); ++i)
            std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                      << std::setw(10) << cloud->points[point_idx_nkn_search[i]].x
                      << std::setw(10) << cloud->points[point_idx_nkn_search[i]].y
                      << std::setw(10) << cloud->points[point_idx_nkn_search[i]].z << " (squared distance: "
                      << std::setw(10) << point_nkn_squared_distance[i] << ")"
                      << std::endl;
    }

    // 半径内近邻搜索
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    const auto radius = 256.0f * rand() / (RAND_MAX + 1.0f);
    std::cout << "Neighbors within radius search at ("
              << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
              << std::setw(10) << searchPoint.x
              << std::setw(10) << searchPoint.y
              << std::setw(10) << searchPoint.z << ") with radius="
              << radius
              << std::endl;

    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                      << std::setw(10) << cloud->points[pointIdxRadiusSearch[i]].x
                      << std::setw(10) << cloud->points[pointIdxRadiusSearch[i]].y
                      << std::setw(10) << cloud->points[pointIdxRadiusSearch[i]].z << " (squared distance: "
                      << std::setw(10) << pointRadiusSquaredDistance[i] << ")"
                      << std::endl;
    }

    return EXIT_SUCCESS;
}
