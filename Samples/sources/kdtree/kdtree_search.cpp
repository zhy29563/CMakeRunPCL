#include <iostream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ctime>
#include <vector>

/*
 * PCL中 K-D TREE 库提供了 K-D TREE 数据结构，基于 FLANN 进行快速最近邻检索。
 * 最近邻检索在匹配、特征描述子计算、邻域特征提取中是非常基础的核心操作。
 */
int main(int argc, char **argv)
{
    // 设置随机种子
    srand(time(NULL));

    // 定义点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 设置参数
    cloud->width = 2000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    // 填充点云
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; // 创建K-D Tree对象
    kdtree.setInputCloud(cloud);            // 向K-D Tree对象中填充点云

    // 创建搜索点
    pcl::PointXYZ search_point;
    search_point.x = rand() * 1024.0f / (RAND_MAX + 1.0f);
    search_point.y = rand() * 1024.0f / (RAND_MAX + 1.0f);
    search_point.z = rand() * 1024.0f / (RAND_MAX + 1.0f);

    /*******************************************************************************************************************
    * K 近邻搜索，获取距离搜索点距离最近的 K 个点
    *******************************************************************************************************************/
    const auto k = 10;             // 指定搜索近邻的数量
    std::vector<int> index_k(k);   // 存储近邻点在原始点云中的索引
    std::vector<float> sdist_k(k); // 存储近邻点到查找点的平方距离

    std::cout << "K nearest neighbor search at ("
              << search_point.x << " "
              << search_point.y << " "
              << search_point.z << ") with K="
              << k
              << std::endl;

    if (kdtree.nearestKSearch(search_point, k, index_k, sdist_k) > 0)
    {
        for (size_t i = 0; i < index_k.size(); ++i)
            std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                      << std::setw(10) << cloud->points[index_k[i]].x
                      << std::setw(10) << cloud->points[index_k[i]].y
                      << std::setw(10) << cloud->points[index_k[i]].z
                      << " (squared distance: " << sdist_k[i] << ")" << std::endl;
    }

    /*******************************************************************************************************************
    * 半径 近邻搜索，获取到搜索点距离小于指定半径的所有近邻点
    *******************************************************************************************************************/
    std::vector<int> index_r;                               // 存储满足条件的点在原始点云中的索引
    std::vector<float> rdist_r;                             // 存储满足条件的点到搜索点的平方距离
    const auto radius = rand() * 256.0 / (RAND_MAX + 1.0f); // 半径值

    std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
              << "Neighbors within radius search at ("
              << std::setw(10) << search_point.x
              << std::setw(10) << search_point.y
              << std::setw(10) << search_point.z
              << ") with radius=" << std::setw(10) << radius << std::endl;

    const auto num = kdtree.radiusSearch(search_point, radius, index_r, rdist_r);
    if (num > 0)
    {
        for (size_t i = 0; i < index_r.size(); ++i)
            std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                      << std::setw(10) << cloud->points[index_r[i]].x
                      << std::setw(10) << cloud->points[index_r[i]].y
                      << std::setw(10) << cloud->points[index_r[i]].z
                      << " (squared distance: " << std::setw(10) << rdist_r[i] << ")" << std::endl;
    }

    return EXIT_SUCCESS;
}
