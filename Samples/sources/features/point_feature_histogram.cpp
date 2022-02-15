#include <iostream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d_omp.h>

/*
 * 点特征直方图描述子（Point Feature Histogram，简称 PFH）
 *
 * 正如点特征表示法所示，表面法线和曲率估计是某个点周围的几何特征基本表示法。虽然计算非常快速容易，但是无法获得太多信息，因为他们只
 * 使用很少的几个参数值来近似表示一个点的K邻域的几何特征。然而大部分场景中包含许多特征点，这些特征点有相同的或非常相近的特征值，因此
 * 采用点特征表示法，其直接结果就减少了全局的特征信息。
 *
 * PFH 计算方式通过参数化查询点与邻域点之间的空间差异，并形成一个多维直方图对点的K邻域几何属性进行描述。直方图所在的高维空间为特征
 * 表示提供了一个可度量的信息空间，对点云对应曲面的6维姿态来说它具有不变性，并且砸死不同的采样密度或邻域的噪声等级下具有鲁棒性。点特
 * 征直方图（PFH）表示法是基于点与其K邻域之间的关系以及他们的法线估计，简而言之，它考虑估计法线方向之间所有的相互作用，试图捕获最好
 * 的样本表面变化情况，以描述样本的几何特征。因此，合成特征超空间取决于每个点的表面法线估计的质量。
 *
 * 对于任意一查询点（Pq），获取在给定半径（r）范围内的所有K个近邻点。现对查询点 + K个近邻点共 K + 1 个点，两两配对，可得到
 * （K + 1）* K / 2 个点对。
 *
 * 对于每一个点对，构建局部UVW坐标系。假设两个点的名称分别为源点(Source Point)与目标点(Target Point)，源点与目标点的法向量分别为
 * Ns与Nt。以源点所在的位置为局部坐标系的原点，方向定义如下：
 * U = Ns
 * V = U @ NormalVector(Pt - Ps) 其中 @ 表示叉乘，NormalVector 表示计算 Ps 到 Pt 向量的单位向量
 * W = U @ V
 *
 * PFH 特征四元组表示如下：
 *  α = V * Nt  其中 * 表示点积
 *  φ = U *NormalVector(Pt - Ps)
 *  θ = arctan(W * Nt, U * Nt)
 *  d = distance(Ps, Pt)  欧式距离
 * 这样就把两个点和它们法线相关的12个参数减少到4个。为每一对点估计PFH四元组使用pcl::computePairFeatures函数
 *
 * 为查询点创建最终的PFH表示，所有的四元组将会以某种统计的方式放进直方图中，这个过程首先把每个特征值划分为B个子区间，并统计落在每个
 * 子区间的数目，因为四分之三的特征在上述中为法线之间的角度计量，在三角化圆上可以将他们的参数值非常容易第归一化到相同的区间内。
 *
 * 在PCL中，默认PFH的实现使用5个区间分类，其中不包括距离，这样就形成了一个125个浮点数元素的特征向量，其保存在一个pcl::PFHSignature125
 * 的点类型中。
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

    // PFH 特征
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>());
    // PFH估计
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);
    pfh.setSearchMethod(tree2);
    pfh.setRadiusSearch(0.02);
    pfh.compute(*pfhs);

    std::cout << "PFH Size: " << pfhs->size() << std::endl;

    return EXIT_SUCCESS;
}