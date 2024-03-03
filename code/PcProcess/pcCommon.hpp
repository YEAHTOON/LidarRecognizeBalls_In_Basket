#ifndef __PC_COMMON_HPP
#define __PC_COMMON_HPP

#include <pcl/io/pcd_io.h>     //pcd读写文件
#include <pcl/point_types.h>   //点类型文件
#include <pcl/point_cloud.h>  
#include <pcl/common/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

#include <time.h>
#include <chrono>
#include <string.h>
#include <sys/time.h>
#include <pthread.h>

class PointClouds
{
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr;    //点云指针

public:
    PointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr ptr) : ptr(ptr)
    {

    }
};

#endif
