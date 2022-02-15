#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <sstream>

#ifdef WIN32
#define SLEEP(x) Sleep((x)*1000)
#endif

/*
 * 八叉树点云压缩
 *
 * 点云由于庞大的数据集组成，这些数据集通过距离、颜色、法线等附加信息来描述空间三位点。此外，点云能以非常告的速率被创建出来
 * ，因此需要占用相当大的存储资源，一旦点云需要存储或者通过速率受限制的通信信道进行传输，提供针对这种数据的压缩方法就变得十
 * 分有用。PCL库提供了点云压缩功能，它允许编码压缩所有类型的点云，包括“无序”点云，它具有无参考点和变化的点尺寸、分辨率、分
 * 布密度和点顺序等结构特征。而且底层八叉树数据结构允许从几个输入源高效地合并点云数据。
 */
int main(int argc, char **argv)
{
    // 点云读取
    pcl::PCLPointCloud2 cloudPcd;
    if (pcl::io::loadPCDFile("../../data/table_scene_lms400.pcd", cloudPcd) != 0)
    {
        std::cout << "Load pcd file is failtrue" << std::endl;
        return EXIT_FAILURE;
    }

    // 点云转换
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(cloudPcd, cloud);
    std::cout << "Raw point cloud size " << cloud.size() << std::endl;

    // 压缩与解压缩
    // 可用压缩配置文件
    //   LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR:  分辨率一立方厘米，无颜色，快速在线编码
    //   LOW_RES_ONLINE_COMPRESSION_WITH_COLOR:     分辨率一立方厘米，有颜色，快速在线编码
    //   MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR:  分辨率五立方毫米，无颜色，快速在线编码
    //   MED_RES_ONLINE_COMPRESSION_WITH_COLOR:     分辨率五立方毫米，有颜色，快速在线编码
    //  HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR:  分辨率一立方毫米，无颜色，快速在线编码
    //  HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR:     分辨率一立方毫米，有颜色，快速在线编码
    //  LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR:  分辨率一立方厘米，无颜色，高效离线编码
    //  LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR:     分辨率一立方厘米，有颜色，高效离线编码
    //  MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR:  分辨率五立方毫米，无颜色，高效离线编码
    //  MED_RES_OFFLINE_COMPRESSION_WITH_COLOR:     分辨率五立方毫米，有颜色，高效离线编码
    // HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR:  分辨率一立方毫米，无颜色，高效离线编码
    // HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR:     分辨率一立方毫米，有颜色，高效离线编码
    //
    // MANUAL_CONFIGURATION:                        允许为高级参数化进行手工配置
    // 为了能完全控制压缩相关的参数，PointCloudCompression类的构造函数可以在初始化时附加压缩参数。请注意，为了能够
    // 进行高级参数化，compressionProfile_arg参数需要被设置成MANUAL_CONFIGURATION
    // PointCloudCompression(compression_Profiles_e compressionProfile_arg,
    //                       bool                   showStatistics_arg,
    //                       const double           pointResolution_arg,
    //                       const double           octreeResolution_arg,
    //                       bool                   doVocelGridDownDownSampling_arg,
    //                       const                  unsigned int iFrameRate_arg,
    //                       bool                   doColorEncoding_arg,
    //                       const unsigned char    colorBitResolution_arg)
    const auto compression_profile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> point_cloud_encoder(compression_profile, false);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> point_cloud_decoder;

    std::stringstream compressedData; // 存储压缩点云的字节流
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = cloud.makeShared();
    point_cloud_encoder.encodePointCloud(cloudPtr, compressedData); // 压缩点云
    std::cout << "encoder point cloud size " << compressedData.str().length() << std::endl;

    point_cloud_decoder.decodePointCloud(compressedData, cloudOut); // 解压缩点云
    std::cout << "decoder point cloud size " << cloudOut->size() << std::endl;

    return EXIT_SUCCESS;
}
