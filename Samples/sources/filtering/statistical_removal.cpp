#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
/*
 * 激光扫描通常会产生密度不均匀的点云数据集。另外，测量中的误差会产生稀疏的利群点，使效果更糟糕。
 * 估计局部点云特征（例如采样点处的法向量或曲率变化率）的运算很复杂，这回导致错误的数值，反过来
 * 有可能导致点云配准等后期处理失败。以下方法可以解决其中部分问题：对于每个点的邻域进行一个统计
 * 分析，并修剪掉那些不符合一定标准的点。此处的稀疏离群点移除方法基于在输入数据中对点到临近点的
 * 距离分布的计算。对每个点，计算它到所有临近点的平均距离。假设得到的结果是一个高斯分布，其形状
 * 由均值喝标准差巨顶，平均距离在标准范围（由全局距离平均值和方差定义）之外的点，可悲定义为离群
 * 点并可从数据集中去除掉。
 * 
 * 计算流程：
 * 1. 计算输入点云的平均值u与方差σ
 * 2. 根据setMeanK成员函数设置的近邻点数量计算其K个距离
 * 3. 根据setSrddevMulThresh设置的方差系数s，判个上一步中的K个距离是否满足：
 *    u - s * σ < distance < u + s * σ
 */
int main (int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ> ("../../data/table_scene_lms400.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);              // 设置在进行统计时考虑查询点近邻点数
    sor.setStddevMulThresh (1.0);   // 设置判断是否为离群点的西格玛系数
    sor.filter (*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    sor.setNegative (true);
    sor.filter (*cloud_filtered);
    writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

    return EXIT_SUCCESS;
}