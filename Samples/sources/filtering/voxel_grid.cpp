#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

/*
 * 体素化网格方法实现降采样，即减少点的数量，减少点云数据，并同时保持点云的形状特征，在提高配准、曲面重建、
 * 形状识别等算法速度中非常实用。PCL实现的VoxelGrid类通过输入的点云数据创建一个三位体素网格(三维立方体)内，
 * 用体素中所有点的重心来近似显示体素中其他的点云。这种方法比用体素中心来逼近的方法更慢，但它对于采样点对
 * 应曲面的表面更准确。
 */

int main (int argc, char **argv)
{
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    pcl::PCDReader reader;
    reader.read ("../../data/table_scene_lms400.pcd", *cloud);

    std::cout << "PointCloud before filtering: " << cloud->width * cloud->height 
              << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f); // 设置滤波时创建的体素大小为1㎝立方体
    sor.filter (*cloud_filtered);

    std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
              << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

    pcl::PCDWriter writer;
    writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
            Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

    return EXIT_SUCCESS;
}