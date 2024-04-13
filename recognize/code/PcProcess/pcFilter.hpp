#ifndef __PC_FILTER_HPP
#define __PC_FILTER_HPP

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include <pcl/io/pcd_io.h>     //pcd读写文件
#include <pcl/point_types.h>   //点类型文件
#include <pcl/point_cloud.h>  
#include <pcl/common/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <time.h>
#include <chrono>
#include <string.h>
#include <sys/time.h>
#include <pthread.h>


/**
 * 点云的滤波器
*/
class pcFilter
{
private:
    void *method;   //方法
    pcl::PointCloud<pcl::PointXYZ>::Ptr target;     //要被处理的点云

public:
    pcFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr target) : method(nullptr), target(target){}
    pcFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr target, void *method) : method(method), target(target){}
    virtual void realize(void){}        //使用滤波器

    void *getMethod(void)
    {
        return method;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr getPC(void)
    {
        return this->target;
    }

    void preset(void *method, pcl::PointCloud<pcl::PointXYZ>::Ptr target)
    {
        this->method = method;
        this->target = target;
    }
};



/**
 * 距离滤波器
*/
class distance_filter : pcFilter
{
private:
    char axis;      //坐标轴
    float max;      //上限
    float min;      //下限
    
public:
    distance_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr target, void *method, char axis, float max, float min):
    pcFilter(target, method), axis(axis), max(max), min(min){}

    virtual void realize(void)
    {
        pcl::PassThrough<pcl::PointXYZ> *method = (pcl::PassThrough<pcl::PointXYZ> *)this->getMethod();
        method->setInputCloud(this->getPC());
        char a[2] = {0};
        sprintf(a, "%c", this->axis);
        method->setFilterFieldName(a);
        method->setFilterLimits(this->min, this->max);
        method->filter(*this->getPC());
    }

    virtual void realize(char axis, float max, float min)
    {
        this->reset(axis, max, min);
        this->realize();
    }

    virtual void realize(pcl::PointCloud<pcl::PointXYZ>::Ptr result)
    {
        pcl::PassThrough<pcl::PointXYZ> *method = (pcl::PassThrough<pcl::PointXYZ> *)this->getMethod();
        method->setInputCloud(this->getPC());
        char a[2] = {0};
        sprintf(a, "%c", this->axis);
        method->setFilterFieldName(a);
        method->setFilterLimits(this->min, this->max);
        method->filter(*result);
    }

    virtual void realize(char axis, float max, float min, pcl::PointCloud<pcl::PointXYZ>::Ptr result)
    {
        this->reset(axis, max, min);
        this->realize(result);
    }

    void reset(char axis, float max, float min)
    {
        this->axis = axis;
        this->max = max;
        this->min = min;
    }

    void reset(char axis, float max, float min, void *method, pcl::PointCloud<pcl::PointXYZ>::Ptr target)
    {
        this->axis = axis;
        this->max = max;
        this->min = min;
        this->preset(method, target);
    }
};


/**
 * 离群点滤波
*/
class outlier_filter : pcFilter
{
private:
    float r;                //领域点数
    float min_num;          //标准差   

public:
    outlier_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr target, void *method, float r, float min_num) 
    : pcFilter(target, method), r(r), min_num(min_num) {}

    virtual void realize(void)
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> *method = (pcl::RadiusOutlierRemoval<pcl::PointXYZ> *)this->getMethod();
        method->setInputCloud(this->getPC());
        method->setRadiusSearch(this->r);     
        method->setMinNeighborsInRadius(this->min_num); 
        method->filter(*this->getPC());
    }

    virtual void realize(float d, int f(float))
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> *method = (pcl::RadiusOutlierRemoval<pcl::PointXYZ> *)this->getMethod();
        method->setInputCloud(this->getPC());
        method->setRadiusSearch(this->r);     
        method->setMinNeighborsInRadius(f(d)); 
        method->filter(*this->getPC());
    }
};


#endif
