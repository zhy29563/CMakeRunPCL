#include <iostream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/*
 * 1. 为什么用一种新的文件格式
 *      PCD文件格式并非白费力气地做重复工作，现有的文件结构因本身组成的原因不支持由PCL库引进N维点类型机制处理过程中
 *      的某些扩展，而PCD文件格式能够很好地补足这一点。PCD不是第一个支持3D点云数据的文件类型，尤其是计算机图形学和
 *      计算机几何学领域，已经创建了很多格式来描述任意多边形和激光扫面仪获取的点云，包括下面几种格式。
 *      ✦ PLY是一种多边形文件格式
 *      ✦ STL是模型文件格式，主要应用于CAD、CAM领域
 *      ✦ OBJ是从几何学上定义的文件格式
 *      ✦ X3D是符合ISO标准的基于XML的文件格式，用于表示3D计算机图形数据
 *  
 *      以上所有的文件格式都有缺点。因为他们是在不同时间为了不同目的所创建的，那时新的传感器技术和算法都还没发明出来。
 * 
 * 2. PCD版本
 *      在点云库1.0版本发布之前，PCD文件格式有不同的修订号。这些修订号用PCD_Vx来标号，代表PCD文件的0.x版本号。然而
 *      PCD中正式发布的PCD文件格式是0.7版本(PCD_V7).
 * 
 * 3. 文件格式头
 *      每一个PCD文件都包含一个文件头，它确定和声明文件中存储的点云数据的某种特性。PCD文件头必须用ASCII码来编码。PCD
 *      文件中指定的每一个文件头字段以及ASCII点数据都用一个新行(\n)分开了，从0.7版本开始，PCD文件头包含下面的这些字段。
 *      ✦ VERSION
 *          指定PCD文件版本
 *      ✦ FIELDS
 *          指定一个点可以有的每一个维度和字段的名字。例如：
 *          FIELDS x y z
 *          FIELDS x y zrgb
 *          FIELDS x y z normal_x mormal_y mormal_z
 *          FIELDS j1 j2 j3 # 矩不变量
 *      ✦ SIZE
 *          用字节数指定每一个维度的大小。例如：
 *          unsigned char  | char               1byte
 *          unsigned short | short              2bytes
 *          unsigned int   | int    | float     4bytes
 *          double                              8bytes
 *      ✦ TYPE
 *          用一个字符指定每一个维度的类型。现在被接受的类型有：
 *          I 表示 有符号类型 int8(char)           | int16(short)           | int32(int)
 *          U 表示 无符号类型 uint8(unsigned char) | uint16(unsigned short) | uint32(unsigned int)
 *          F 表示 浮点类型
 *      ✦ COUNT
 *          指定每一个维度包含的元素数目。例如，x这个数据通常有一个元素，但是像VFH这样的特征描述子有308个。实际上这是
 *          在给每一点引入N维直方图描述符的方法，把他们当作单个的连续存储块。默认情况下，如果没有COUNT，所有维度的数目
 *          被设置成1。
 *      ✦ WIDTH
 *          用点的数量表示点云数据集的宽度。根据是有序点云还是无序点云，WIDTH有两层解释。
 *          1) 它能确定无序数据集的点云中点的个数
 *          2) 它能确定有序点云数据集的宽度(一行中点的数目)
 *          注意：有序点云数据集，意味着点云是类似图像(或者矩阵)的结构，数据分为行和列。这种点云的实例包括立体摄像机和
 *          时间飞行摄像机生成的数据。有序数据集的优势在于，预先了解相邻点的关系，邻域操作更加高效，这样就加速了计算并
 *          降低了PCL中某些算法的成本。
 *      ✦ HEIGHT
 *          用点的数目表示点云数据集的高度。类似于WIDTH，HEIGHT也有两层解释。
 *          1) 它表示有序点云数据集的高度(行的总数)
 *          2) 对于无序数据集，它被设置成1(被用来检查一个数据集是有序还是无序)
 *      ✦ VIEWPORT
 *          指定数据集中点云的获取视点。VIEWPORT有可能在不同坐标系之家转换的时候应用，在辅助获取其他特征时也比较有用，
 *          例如曲面法线，在判断方向一致性时，需要指定视点的方位。视点信息被指定为平移(txtytz) + 四元素(qwqxqyqz).
 *          默认值如下：
 *          VIEWPORT 0 0 0 1 0 0 0
 *      ✦ POINTS
 *          指定点遇见你中点的总数。从0.7版本开始，该字段就显得有点多余了，因此有可能在将来的版本中将它移除。
 *      ✦ DATA
 *          指定存储点云数据的数据类型。从0.7版本开始，支持两种数据类型：ASCII和二进制。
 */

int main (int argc, char **argv)
{
    pcl::PCLPointCloud2 cloud;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int pcd_version;
    int data_type;
    unsigned int data_idx;
    int offset = 0;

    pcl::PCDReader reader;
    reader.readHeader ("../../data/table_scene_lms400.pcd", cloud, origin, orientation, pcd_version, data_type, data_idx, offset);
    
    std::cout << "cloud header     : " << cloud.header << std::endl;
    std::cout << "cloud width      : " << cloud.width << std::endl;
    std::cout << "cloud height     : " << cloud.height << std::endl;
    std::cout << "cloud point_step : " << cloud.point_step << std::endl;
    std::cout << "cloud row_step   : " << cloud.row_step << std::endl;

    std::cout << "cloud fields     : " << std::endl;
    std::cout << "cloud fields size: " << cloud.fields.size() << std::endl;
    for(pcl::PCLPointField& field: cloud.fields)
        std::cout << field << std::endl;
    
    std::cout << "cloud data size   : " << cloud.data.size() << std::endl;
    

    std::cout << "origin.x         : " << origin.x() << std::endl;
    std::cout << "origin.y         : " << origin.y() << std::endl;
    std::cout << "origin.z         : " << origin.z() << std::endl;
    std::cout << "orientation.x    : " << orientation.x() << std::endl;
    std::cout << "orientation.y    : " << orientation.y() << std::endl;
    std::cout << "orientation.z    : " << orientation.z() << std::endl;
    std::cout << "orientation.w    : " << orientation.w() << std::endl;
    std::cout << "pcd_version      : " << pcd_version << std::endl;
    std::cout << "data_type        : " << data_type << std::endl;
    std::cout << "data_idx         : " << data_idx << std::endl;

    return EXIT_SUCCESS;
}