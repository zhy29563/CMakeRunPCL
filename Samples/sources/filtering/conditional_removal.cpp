#include <iostream>
#include <iomanip>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>

/*
 * 一次删除满足对输入的点云设定的一个或多个条件指标的所有数据
 */
int main (int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width  = 5;
    cloud->height = 1;
    cloud->resize (cloud->width * cloud->height);

    for (auto& point: *cloud)
    {
        point.x = rand () / (RAND_MAX + 1.0f);
        point.y = rand () / (RAND_MAX + 1.0f);
        point.z = rand () / (RAND_MAX + 1.0f);
    }

    // build the condition
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.5)));

    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(true);     // 参数为true， 则输出点云的结构保持不变，被移除的点将被设置为NAN
                                        // 参数为false, 则输出点云的数量发生改变，不满足条件的点被直接删除

    // apply filter
    condrem.filter (*cloud_filtered);

    std::cout << "Cloud before filtering: " << std::endl;
    for (const auto& point: *cloud)
        std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                  << std::setw(10) << point.x 
                  << std::setw(10) << point.y
                  << std::setw(10) << point.z << std::endl;

    // display pointcloud after filtering
    std::cout << "Cloud after filtering: " << std::endl;
    for (const auto& point: *cloud_filtered)
        std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                  << std::setw(10) << point.x 
                  << std::setw(10) << point.y
                  << std::setw(10) << point.z << std::endl;

    // 移除NAN点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_nan(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::Indices indices;
    pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_non_nan, indices);

    std::cout << "Cloud after removing nan: " << std::endl;
    for (const auto& point: *cloud_non_nan)
        std::cout << std::setprecision(3) << std::setiosflags(std::ios::fixed | std::ios::right)
                  << std::setw(10) << point.x 
                  << std::setw(10) << point.y
                  << std::setw(10) << point.z << std::endl;

    return EXIT_SUCCESS;
}