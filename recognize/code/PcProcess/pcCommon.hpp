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
#include <fstream>
#include <sys/time.h>

// 非常好
class PointClouds
{
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr;    //点云指针

public:
    PointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr ptr) : ptr(ptr)
    {

    }
};


// 导出点云数据
class PCs_outer
{
public:
    PCs_outer(const char *out_path)
    {
        memset(this->out_path, 0, 128);
        memcpy(this->out_path, out_path, strlen(out_path));
    }

    // 保存点云
    void store_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr thePC, const char * name)
    {
        //创建要写入的字符串
        char data[100000] = {0};
        char *data_pointer = data;

        //遍历每一个点，加入字符串
        for(auto it = thePC->begin(); it != thePC->end(); ++it)
        {
            //获得点
            pcl::PointXYZ p = *it;      

            //把点的信息转换为字符串
            char point_data[128] = {0};
            sprintf(point_data, "%f %f %f\n", p.x, p.y, p.z);

            //尾部插入新数据
            memcpy(data_pointer, point_data, strlen(point_data));

            //移动指针
            data_pointer = data_pointer + strlen(point_data);       //改变指针
        }

        //打开文件，准备写入
        std::fstream f;
        char file_path[1024] = {0}; sprintf(file_path, "%s/%s.pts", out_path, name);
        f.open(file_path, std::ios::out);

        //写入
        f << (const char *)data;
        
        //关闭
        f.close();
    }

private:
    char out_path[128];
};


#endif
