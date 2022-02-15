#include <iostream>
#include <iomanip>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

/*
 * 对指定的某一维度实行一个简单的滤波，即去掉在用户指定范围内部（外部）的点
 */

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    
    // Fill in the cloud data
    cloud->width  = 5;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

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

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 0.5);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

    std::cout << "Cloud after filtering: " << std::endl;
    for (const auto& point: *cloud_filtered)
        std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                  << std::setw(10) << point.x 
                  << std::setw(10) << point.y
                  << std::setw(10) << point.z << std::endl;

    return EXIT_SUCCESS;
}