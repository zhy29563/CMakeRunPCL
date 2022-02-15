#include <iostream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>

/*
 * 快速点特征直方图描述子（Fast Point Feature Histogram，简称 FPFH）
 *
 * 已知点云P中有N个点，那么他的点特征直方图（PFH）的理论计算复杂度是O(NKK)，其中K是点云P中每个点p计算特征向量时考虑的邻域数量。对于
 * 实时或接近实时应用中，密集点云的点特征直方图的计算，时一个主要的性能瓶颈。
 *
 * 快速点特征直方图描述子将算法的复杂度降低到了O（NK），单仍然保留了PFH大部分的识别特性。
 *
 * 【原理】
 * 第一步，对于每一个查询点Pq，计算这个点和它的邻域点之间的一个元素α，φ，θ，这一步结果称之为简化的点特征直方图（Simplified Point
 * Feature Histograms，简称SPFH）
 * 第二步，重新确定每个点的K邻域，使用邻近的SPFH值来计算Pq的最终直方图，即FPFH
 *   FPFH（Pq） = SPFH(Pq) + Sum(SPFH(Pk)/Wk)/K
 * 权重Wk在一些给定的度量空间中，表示查询点Pq和其临近点Pk之间的距离，因此可以量评定一对点（Pq,Pk），但是如果需要的话，也可以把用Wk
 * 另一种度量来表示。
 *
 *
 * PFH与FPFH的区别
 * 1. FPFH没有对全部互联Pq点的所有邻近点的计算参数进行统计，因此可能漏掉了一些重要的点对，而这些漏掉的点对可能对捕获查询点周围的几
 *    何特征有贡献
 * 2. PFH特征模型是在查询点周围的一个精确的邻域半径内，而FPFH还包括半径R范围以外的额外点对(不过在2R半径内)
 * 3. 因为重新权重计算的方式，所以FPFH结合SPFH值，重新捕获邻近重要点对的几何信息
 * 4. 由于大大地降低了FPFH的整体复杂性，因此FPFH有可能使用在实时应用中
 * 5. 通过分解三元组，简化了合成的直方图。也就是简单生成D分离特征直方图对每个特征维度来单独沪指，并把他们连接在一起
 *
 * 在PCL中，默认FPFH实现使用11个统计子区间，特征中覅那个图被分别计算然后合并得到出了浮点值的一个33元素的特征向量，这些保存至在一个
 * pcl::FPFHSignature33点类型中
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

    // FPFH 特征
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

    // FPFH估计
    // pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.02);
    fpfh.compute(*fpfhs);

    std::cout << "FPFH Size: " << fpfhs->size() << std::endl;

    return EXIT_SUCCESS;
}