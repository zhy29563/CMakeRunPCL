#include <iostream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
    // 定义点云
    pcl::PointCloud<pcl::PointXYZ> cloud_a;

    // 设置点云属性
    cloud_a.width = 5;
    cloud_a.height = 1;
    cloud_a.points.resize(cloud_a.width * cloud_a.height);

    // 点云数据填充
    for (size_t i = 0; i < cloud_a.points.size(); ++i)
    {
        cloud_a.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_a.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_a.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    // 定义点云
    pcl::PointCloud<pcl::Normal> cloud_b;

    // 设置点云属性
    cloud_b.width = 5;
    cloud_b.height = 1;
    cloud_b.points.resize(cloud_b.width * cloud_b.height);

    // 点云数据填充
    for (size_t i = 0; i < cloud_b.points.size(); ++i)
    {
        cloud_b.points[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_b.points[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_b.points[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    // 定义点云
    pcl::PointCloud<pcl::PointNormal> cloud_c;

    // 点云合并
    pcl::concatenateFields(cloud_a, cloud_b, cloud_c);

    // 点云显示
    std::cout << "Cloud C: " << std::endl;
    for (const auto& point: cloud_c)
        std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                  << std::setw(10) << point.x 
                  << std::setw(10) << point.y
                  << std::setw(10) << point.z 
                  << std::setw(10) << point.normal_x
                  << std::setw(10) << point.normal_y
                  << std::setw(10) << point.normal_z << std::endl;

    return EXIT_SUCCESS;
}