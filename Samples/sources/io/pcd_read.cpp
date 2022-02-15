#include <iostream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return EXIT_FAILURE;
    }

    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    for (const auto &point : *cloud)
        std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                  << std::setw(10) << point.x
                  << std::setw(10) << point.y
                  << std::setw(10) << point.z << std::endl;
    return EXIT_SUCCESS;
}