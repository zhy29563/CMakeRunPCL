#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h>

/*
 * 如何从深度图像中提取边界（从前景跨越到背景的位置定义为边界）
 * 我们对三种类型的点集感兴趣：
 * 物体边界：这时物体最外层和阴影边界的可见点集
 * 阴影边界：毗连于遮挡的背景上的点集
 * Veil点集：在被遮挡物边界和阴影边界之间的内插点，它们是由激光雷达获取的3D距离数据中典型的数据类型
 */

 /***********************************************************************************************************************
  * 类型别名定义
  **********************************************************************************************************************/
typedef pcl::PointXYZ PointType;

/***********************************************************************************************************************
 * 全局参数
 **********************************************************************************************************************/
float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

/***********************************************************************************************************************
 * 全局函数
 **********************************************************************************************************************/
void printUsage(const char* progName)
{
    std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
        << "Options:\n"
        << "-------------------------------------------\n"
        << "-r <float> angular resolution in degrees (default " << angular_resolution << ")\n"
        << "-c <int>   coordinate frame (default " << (int)coordinate_frame << ")\n"
        << "-m         Treat all unseen points to max range\n"
        << "-h         this help\n"
        << "\n\n";
}

/***********************************************************************************************************************
 * 主函数调用
 **********************************************************************************************************************/
int main(int argc, char** argv)
{
    /*******************************************************************************************************************
     * 参数解析
     ******************************************************************************************************************/
    if (pcl::console::find_argument(argc, argv, "-h") >= 0)
    {
        printUsage(argv[0]);
        return EXIT_SUCCESS;
    }

    if (pcl::console::find_argument(argc, argv, "-m") >= 0)
    {
        setUnseenToMaxRange = true;
        std::cout << "Setting unseen values in range image to maximum range readings.\n";
    }

    int tmp_coordinate_frame;
    if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
    {
        coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
        std::cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
    }

    if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0)
        std::cout << "Setting angular resolution to " << angular_resolution << "deg.\n";
    angular_resolution = pcl::deg2rad(angular_resolution);

    /*******************************************************************************************************************
     * 创建点云
     ******************************************************************************************************************/
    pcl::PointCloud<PointType> point_cloud;
    for (float x = -0.5f; x <= 0.5f; x += 0.01f)
    {
        for (float y = -0.5f; y <= 0.5f; y += 0.01f)
        {
            PointType point;
            point.x = x;
            point.y = y;
            point.z = 2.0f - y;
            point_cloud.points.push_back(point);
        }
    }
    point_cloud.width = (int)point_cloud.points.size();
    point_cloud.height = 1;

    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());

    // 从点云创建深度图像的参数
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;

    // 从点云创建深度图像
    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;
    range_image.createFromPointCloud(point_cloud,
        angular_resolution,
        pcl::deg2rad(360.0f),
        pcl::deg2rad(180.0f),
        scene_sensor_pose,
        coordinate_frame,
        noise_level,
        min_range,
        border_size);

    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    range_image.integrateFarRanges(far_ranges);
    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange();

    /*******************************************************************************************************************
     * 显示点云
     ******************************************************************************************************************/
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    viewer.addCoordinateSystem(1.0f);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(point_cloud.makeShared(), 0, 0, 0);
    viewer.addPointCloud(point_cloud.makeShared(), color_handler, "original point cloud");

    /*******************************************************************************************************************
     * 提取边界
     ******************************************************************************************************************/
    pcl::RangeImageBorderExtractor border_extractor(&range_image);
    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    border_extractor.compute(border_descriptions);

    /*******************************************************************************************************************
     * 显示边界
     ******************************************************************************************************************/
    pcl::PointCloud<pcl::PointWithRange> border_points;
    pcl::PointCloud<pcl::PointWithRange> veil_points;
    pcl::PointCloud<pcl::PointWithRange> shadow_points;
    for (int y = 0; y < (int)range_image.height; ++y)
    {
        for (int x = 0; x < (int)range_image.width; ++x)
        {
            int offset = y * range_image.width + x;
            pcl::BorderDescription description = border_descriptions.points[offset];
            pcl::PointWithRange range = range_image.points[offset];
            if (description.traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
                border_points.points.push_back(range);

            if (description.traits[pcl::BORDER_TRAIT__VEIL_POINT])
                veil_points.points.push_back(range);

            if (description.traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
                shadow_points.points.push_back(range);
        }
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_color_handler(border_points.makeShared(), 0, 255, 0);
    viewer.addPointCloud<pcl::PointWithRange>(border_points.makeShared(), border_color_handler, "border points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_color_handler(veil_points.makeShared(), 255, 0, 0);
    viewer.addPointCloud<pcl::PointWithRange>(veil_points.makeShared(), veil_color_handler, "veil points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_color_handler(shadow_points.makeShared(), 0, 255, 255);
    viewer.addPointCloud<pcl::PointWithRange>(shadow_points.makeShared(), shadow_color_handler, "shadow points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");

    /*******************************************************************************************************************
     * 显示深度图
     ******************************************************************************************************************/
    pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
    range_image_borders_widget = pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(range_image,
        -std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        false,
        border_descriptions,
        "Range image with borders");

    while (!viewer.wasStopped())
    {
        range_image_borders_widget->spinOnce();
        viewer.spinOnce();
        pcl_sleep(0.01);
    }

    return EXIT_SUCCESS;
}