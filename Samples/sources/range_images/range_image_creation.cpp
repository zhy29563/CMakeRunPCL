#include <iostream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>

/*
 * 深度图像是指将图像采集器到场景中各点的距离最为像素值的图像，它直接反映了劲舞可见表面的集合形状，利用它可以很方便地解决3D目标描述
 * 中的许多问题。
 * 深度图像金国坐标转换可以计算为点云数据，有规则及必要信息的点云数据也可以反算为深度图像数据。
 */
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ> pointCloud;
    for (float y = -0.5f; y <= 0.5f; y += 0.01f)
    {
        for (float z = -0.5f; z <= 0.5f; z += 0.01f)
        {
            pcl::PointXYZ point;
            point.x = 2.0f - y;
            point.y = y;
            point.z = z;
            pointCloud.push_back(point);
        }
    }
    pointCloud.width = pointCloud.size();
    pointCloud.height = 1;

    // 定义X与Y方向相邻的像素点所对应的光束之间的角度差
    float angularResolution = (float)(pcl::deg2rad(1.000f)); //   1.0 degree in radians

    float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
    float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f)); // 180.0 degree in radians
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);

    // 用于说明深度图构建系统的坐标系
    // CAMERA_FRAME：X轴向右，Y轴向下，Z轴向前
    // LASER_FRAME ：X轴向前，Y轴向左，Z轴向上
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

    // noise_level = 0 是指只用一个归一化的Z缓冲器来创建深度图，但是如果想让临近点集都落在同一个像素单元，可以设置一个较高的指值。
    // 例如noise_level = 0.05，可以理解为，深度距离值是通过查询点半径为5cm的圆内包含的点以平均计算而得到的。
    float noiseLevel = 0.00;

    // 如果min_range > 0，则所有模拟器所在位置半径为min_range范围内的临近点都将被忽略，即盲区。
    float minRange = 0.0f;

    // 在裁剪图像时，如果border_size > 0，将在图像周围留下当前视点不可见的边界点
    int borderSize = 1;

    // 构建深度图
    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose,
                                    coordinate_frame, noiseLevel, minRange, borderSize);
    std::cout << rangeImage << "\n";

    return EXIT_SUCCESS;
}