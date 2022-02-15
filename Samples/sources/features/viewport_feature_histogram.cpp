#include <iostream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d_omp.h>

/*
 * 视点特征直方图描述子（Viewport Feature Histogram，简称 VFH）
 *
 * 该特征直方图是一个种新的特征表示形式，应用在点云聚类识别和六自由度位姿估计问题。
 *
 * 【原理】
 * 视点特征直方图源于FPFH。由于它的速度和识别力，我们决定利用FPFH强大的识别力，但是为了是构造的特征保持缩放不变性的性质同时，还要区
 * 分不同的位姿，计算时需要考虑加入视点变量。我们做了两种计算来构造特征，以应用与目标识别和位姿估计。
 * 1. 扩展FPFH，使其利用整个点云对象来i进行计算估计，在计算FPFH时以为题中心点与物体表面其他所有点之间的点对作为计算单元
 * 2. 添加视点方向与每个点估计法线之间额外的统计信息，为了达到这个目的，我们的关键想法是在FPFH计算中将视点方向变量直接融入相对法线
 *    角计算当中
 *
 * 通过统计视点方向与每个法线之间角度的直方图来计算视点相关的特征分量。注意：并不是每条法线的视角，因为法线的视角在尺度变换下具有可
 * 变性，我们值得是平移视点到查询点后的视点方向和每条法线间的角度。第二组 特征分量就是PFH中讲述的三个角度，只是现在测量的是在中心点
 * 的视点方向和每条表面法线之间的角度。
 *
 * 在PCL中，默认的VFH的实现使用45个子区间进行统计，面对视点分量要使用128个子区间进行统计，这样VFH就由一共308个浮点数组成阵列。
 */

int main(int argc, char **argv)
{
    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    int ret = pcl::io::loadPCDFile("../../data/table_scene_lms400.pcd", *cloud);
    if (ret == -1)
    {
        std::cout << "Load point cloud is fail." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Cloud Width : " << cloud->width << std::endl;
    std::cout << "Cloud Height: " << cloud->height << std::endl;
    std::cout << "Cloud Size  : " << cloud->size() << std::endl;

    // K-D tree 对象，用于邻域搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    // 法向量
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03); // 使用半径在查询点周围3厘米范围内的所有邻元素
    ne.compute(*normals);     // 计算特征值

    std::cout << "Computer normal is completed." << std::endl;

    // VFH 特征
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>());
    // VFH估计
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud(cloud);
    vfh.setInputNormals(normals);
    vfh.setSearchMethod(tree2);
    vfh.setRadiusSearch(0.05);
    vfh.compute(*vfhs);

    std::cout << "VFH Size: " << vfhs->size() << std::endl;

    return EXIT_SUCCESS;
}