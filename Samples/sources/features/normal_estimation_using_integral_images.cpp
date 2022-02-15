#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

/*
 * 积分图法线估计
 * 该方法仅适用于有序点云的法线估计。
 */
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../../data/table_scene_mug_stereo_textured.pcd", *cloud);
    std::cout << "Cloud Width : " << cloud->width << std::endl;
    std::cout << "Cloud Height: " << cloud->height << std::endl;

    // 法线估计结果
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // 积分图法线估计
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    // 设置法线估计方法，可使用以下几种方法
    // enum NormalEstimationMethod
    // {
    //      COVARIANCE_MATRIX
    //      AVERAGE_3D_GRADIENT
    //      AVERAGE_DEPATH_CHANGE
    // }
    // COVARIANCE_MATRIX: 该模式从具体某个点的局部邻域的协方差矩阵创建9个积分图，来计算这个点的法线
    // AVERAGE_3D_GRADIENT: 创建了6个积分图来计算水平和垂直方向平滑后的三位梯度，并使用两个梯度间的向量积计算法线
    // AVERAGE_DEPATH_CHANGE: 只创建一个单一积分图，并从平均深度变化计算法线
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f); // 设置最大深度变化稀疏
    ne.setNormalSmoothingSize(10.0f);  // 优化法线方向时考虑邻域大小
    ne.setInputCloud(cloud);           // 输入点云，必须是有序点云
    ne.compute(*normals);

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}
