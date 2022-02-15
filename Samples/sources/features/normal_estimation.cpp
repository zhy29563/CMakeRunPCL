#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>

/*
 * 估计一个点云的表面法线
 *
 * 表面法线是几何体表面的重要属性，在很多领域都有大量应用，例如：在进行光照渲染时产生符合可视习惯的效果时需要
 * 表面法线信息才能正常进行，对于一个已知的集合体表面，根据垂直于点表面的矢量，因此推断表面某一点的法线方向通
 * 常比较简单。然而，由于我们获取的点云数据集在真实物体的表面表现为一组定点样本，这样就会有两种解决方法。
 *
 * 1. 使用曲面重建技术，从获取的点云数据集中得到采样点对应的曲面，然后从曲面模型收纳柜计算表面法线
 * 2. 直接从点云数据集中近似推断表面法线
 *
 * 确定表面一点法线的问题近似于估计表面的一个相切面法线的问题，进而变成一个最小二乘法平面拟合估计问题。因此估
 * 计表面法线的解决方案就变成了一个分析洗一个协方差矩阵的特征矢量和特征值，这个协方差矩阵从查询点的近邻元素中
 * 创建。更具体的说，对于每一个点Pi，对应的协方差矩阵C如下：
 *      C = 1/K * SUM[(Pi - Pu) * (Pi - Pu)T]  i ∈ [1, K]
 *      C * Vj = λj * Vj                       j ∈ {0, 1, 2}
 *
 * 在PCL内估计一点集对应的协方差矩阵，可以调用以下函数实现：
 * Eigen::Matrix3f covariance_matrix;                                   协方差矩阵
 * Eigen::Vector4f xyz_centroid;                                        质心对象
 * compute3DCentroid (cloud, xyz_centroid);                             估计质心坐标
 * computeCovarianceMatrix (cloud, xyz_centroid, covariance_matrix);    计算3X3协方差矩阵
 */
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../../data/table_scene_lms400.pcd", *cloud);

    // 输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // 邻域搜索算法
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    /*
     * 法线估计类NormalEstimation内部执行以下操作：
     * 对于点云P中的每个点Pi
     *  1. 得到Pi点的最近邻元素
     *  2. 计算Pi点的表面法线N
     *  3. 检查N的方向是否一致指向视点，如果不是则翻转（默认视点为(0,0,0)）,可使用setViewPoint函数就行调整
     */

    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);   // 使用半径在查询点周围3厘米范围内的所有邻元素
    ne.compute(*cloud_normals); // 计算特征值

    // 法线可视化
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    return EXIT_SUCCESS;
}