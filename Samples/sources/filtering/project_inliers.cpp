#include <iostream>
#include <iomanip>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

// 将点投影到参数化模型，此处的模型为一个平面（ax + by + cz + d = 0）
// 如何确定一个点是内点或外点？
int main (int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

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

    std::cout << "Cloud before projection: " << std::endl;
    for (const auto& point: *cloud)
        std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                  << std::setw(10) << point.x 
                  << std::setw(10) << point.y
                  << std::setw(10) << point.z << std::endl;

    // Create a set of planar coefficients with X=Y=0,Z=1
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = 0;    // a
    coefficients->values[1] = 0;    // b
    coefficients->values[2] = 1.0;  // c
    coefficients->values[3] = 0;    // d

    // 类 Projectlnliers 使用一个模型和一组的内点的索引，将内点投影到模型形成新的一个独立点云
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setInputCloud (cloud);
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setModelCoefficients (coefficients);
    proj.setCopyAllData(false);
    proj.filter (*cloud_projected);

    std::cout << "Cloud after projection: " << std::endl;
    for (const auto& point: *cloud_projected)
        std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                  << std::setw(10) << point.x 
                  << std::setw(10) << point.y
                  << std::setw(10) << point.z << std::endl;

    return EXIT_SUCCESS;
}