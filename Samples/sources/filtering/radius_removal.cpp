#include <iostream>
#include <iomanip>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

/*
 * 在指定的半径范围内至少具有指定数量的近邻，不满足条件的点将被剔除
 */
int main (int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width  = 10;
    cloud->height = 1;
    cloud->resize (cloud->width * cloud->height);

    for (auto& point: *cloud)
    {
        point.x = rand () / (RAND_MAX + 1.0f);
        point.y = rand () / (RAND_MAX + 1.0f);
        point.z = rand () / (RAND_MAX + 1.0f);
    }

    std::cout << "Cloud before filtering: " << std::endl;
    for (const auto& point: *cloud)
        std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                  << std::setw(10) << point.x 
                  << std::setw(10) << point.y
                  << std::setw(10) << point.z << std::endl;

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);        
    outrem.setRadiusSearch(0.5);            // 设置在0.5半径的范围内查找邻近点
    outrem.setMinNeighborsInRadius (2);     // 设置查询点的邻近点集数小于2的删除
    outrem.setKeepOrganized(false);         // 直接删除不满足条件的点
    outrem.filter (*cloud_filtered);

    
    // display pointcloud after filtering
    std::cout << "Cloud after filtering: " << std::endl;
    for (const auto& point: *cloud_filtered)
        std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                  << std::setw(10) << point.x 
                  << std::setw(10) << point.y
                  << std::setw(10) << point.z << std::endl;

    return EXIT_SUCCESS;
}