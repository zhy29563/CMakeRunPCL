#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

/*
 * 用以获取2D封闭多边形内部或者外部的点云
 */

int main(int argc, char **argv)
{
	// 点云读取
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	reader.read("../../data/pig.pcd", *cloud);

	// 多边形边界点
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	boundingbox_ptr->push_back(pcl::PointXYZ(0.1, 0.1, 0));
	boundingbox_ptr->push_back(pcl::PointXYZ(0.1, -0.1, 0));
	boundingbox_ptr->push_back(pcl::PointXYZ(-0.1, 0.1, 0));
	boundingbox_ptr->push_back(pcl::PointXYZ(-0.1, -0.1, 0));
	boundingbox_ptr->push_back(pcl::PointXYZ(0.15, 0.1, 0));

	// 创建凸包
	pcl::ConvexHull<pcl::PointXYZ> hull;
	hull.setInputCloud(boundingbox_ptr);
	hull.setDimension(2); // 设置凸包维度

	// 创建多边形
	std::vector<pcl::Vertices> polygons;
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
	hull.reconstruct(*surface_hull, polygons);

	// 多边形滤波
	pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropHull<pcl::PointXYZ> bb_filter;
	bb_filter.setDim(2);				  // 设置维度
	bb_filter.setInputCloud(cloud);		  // 设置需要滤波的点云
	bb_filter.setHullIndices(polygons);	  // 输入封闭多边形的顶点
	bb_filter.setHullCloud(surface_hull); // 输入封闭多边形的形状
	bb_filter.filter(*objects);			  // 执行CropHull滤波，存储结果到objects

	// 创建可视化对象
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("crophull display"));
	// 设置窗口背景色
	viewer->setBackgroundColor(1.0, 1.0, 1.0);

	// 显示原始点云与多边形
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.33, 1, v1);
	viewer->setBackgroundColor(1.0, 1.0, 1.0, v1);
	viewer->addPointCloud(cloud, "cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0, 0, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	viewer->addPolygon<pcl::PointXYZ>(surface_hull, 0, .069 * 255, 0.2 * 255, "backview_hull_polyline1", v1);

	// 显示多边形
	int v2(0);
	viewer->createViewPort(0.33, 0.0, 0.66, 1, v2);
	viewer->setBackgroundColor(1.0, 1.0, 1.0, v2);
	viewer->addPointCloud(surface_hull, "surface_hull", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0, 0, "surface_hull");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "surface_hull");
	viewer->addPolygon<pcl::PointXYZ>(surface_hull, 0, .069 * 255, 0.2 * 255, "backview_hull_polyline", v2);

	int v3(0);
	viewer->createViewPort(0.66, 0.0, 1, 1, v3);
	viewer->setBackgroundColor(1.0, 1.0, 1.0, v3);
	viewer->addPointCloud(objects, "objects", v3);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0, 0, "objects");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "objects");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(1000);
	}

	return EXIT_SUCCESS;
}