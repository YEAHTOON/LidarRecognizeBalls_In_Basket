#ifndef __GET_PC_
#define __GET_PC_

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
#include <pcl/io/pcd_io.h>     //pcd读写文件
#include <pcl/point_types.h>   //点类型文件
#include <pcl/point_cloud.h>  
#include <pcl/common/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <time.h>
#include <chrono>
#include <string.h>
#include <sys/time.h>
#include <pthread.h>
#include <semaphore.h>




/**
 * 点云帧
 * 
 * begin_time代表了每一帧的起始时刻，利用此判断帧对应的持续时间。
 * 持续时间超过设定值(period_us)后，会导出所有点云并释放前一帧点云，重置起始时刻，为新的一帧做准备。
 * max_point_num是最大点云数量，超过这个数量也会立刻释放点云、重置起始时刻。
 * 
 * storedPoints_num是当前点云数量。
 * 
 * updating是该类最重要的方法，会将更新好的帧导出至缓冲区。
*/
class PointCloud_Frame
{
private:
    //静态的，不需要改变
    float period_us;            //最大采集周期(ms)
    int max_point_num;          //最大点数量

    //动态的，总是在改变
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc;             //更新中的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_buff;    //输出缓冲区
    pthread_mutex_t *output_buff_mutex;
    long long begin_time;           //计数起始时刻
    int storedPoints_num;
    int add_count;                  //添加点云包的计数，每到10的倍数就可触发判断
    sem_t *update_sem;
    

public:

    //初始化点云及其参数、帧起始时刻
    PointCloud_Frame(int max_point_num, int period_us) 
    : period_us(period_us), max_point_num(max_point_num), pc(nullptr), output_buff(nullptr), output_buff_mutex(nullptr), begin_time(0), storedPoints_num(0), add_count(0)
    {
        this->pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);     //点云初始化

        //获得当前时刻,并将当前时刻赋值给更新初始时刻
        long long present_time = getMicroTime();
        this->begin_time = present_time;
    }

    //设置缓冲区的初始化
    PointCloud_Frame(int max_point_num, int period_us, pcl::PointCloud<pcl::PointXYZ>::Ptr output_buff, pthread_mutex_t *output_buff_mutex) 
    : period_us(period_us), max_point_num(max_point_num), pc(nullptr), output_buff(output_buff), output_buff_mutex(output_buff_mutex), begin_time(0), storedPoints_num(0), add_count(0)
    {
        this->pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);     //点云初始化

        //获得当前时刻
        long long present_time = getMicroTime();

        //将当前时刻赋值给初始时刻
        this->begin_time = present_time;

        this->output_buff = output_buff;
        this->output_buff_mutex = output_buff_mutex;
    }

    //增加点云中的点
    void add(LivoxLidarCartesianHighRawPoint *newPoints, int new_PointsNum, float *offset)
    {
        //对于每一个点云
        for(int i = 0; i < new_PointsNum; i++)
        {
            this->pc->points.push_back(pcl::PointXYZ(newPoints[i].x / 1000.0f + offset[0], 
            newPoints[i].y / 1000.0f + offset[1], newPoints[i].z / 1000.0f + offset[2]));      //添加
            this->storedPoints_num++;       //点云数量增加
        }
    }

    //获得当前时刻，以微妙计
    long long getMicroTime(void)
    {
        // auto present_time_source = std::chrono::system_clock::now();
        // long long present_time = std::chrono::duration_cast<std::chrono::microseconds>(present_time_source.time_since_epoch());
        struct timeval present_time = {0};
        gettimeofday(&present_time, NULL);

        return present_time.tv_usec;
    }


    //重置帧
    void reset(void)
    {
        this->pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);     //点云初始化,智能指针自动释放先前空间
        this->begin_time = getMicroTime();      //重置时刻
        this->storedPoints_num = 0;             //计数重置为0
        this->add_count = 0;                    //循环次数重置为0
    }


    //一帧完成的判断函数
    bool frame_done(void)
    {
        //判断点数量
        if(this->storedPoints_num > this->max_point_num)
        {
            return true;
        }

        //判断时间
        long long pass_time = getMicroTime() - this->begin_time;
        (pass_time < 0) ? (pass_time = pass_time + 1000000) : (pass_time = pass_time);
        // std::cout << pass_time << std::endl;
        if(pass_time > period_us)
        {
            return true;
        }
 
        return false;
    }

    //设置缓冲区
    void set_outputBuff(pcl::PointCloud<pcl::PointXYZ>::Ptr buff, pthread_mutex_t *buff_mutex)
    {
        this->output_buff = buff;
        this->output_buff_mutex = buff_mutex;
    }

    //设置更新信号量
    void set_updatesem(sem_t *updateSem)
    {
        this->update_sem = updateSem;
    }

    //导出缓冲区
    void output(void)
    {
        copyPointCloud(*this->pc, *this->output_buff);
        // std::cout << "refresh" << pc->points.size() << std::endl;
    }

    /**
     * 正在更新帧，如果更新完成数据传输至缓冲区
    */
    void updating(LivoxLidarCartesianHighRawPoint *newPoints, int new_PointsNum, float *offset)
    {
        this->add(newPoints, new_PointsNum, offset);    //增添点云

        //传输十个包判断一次
        if(this->add_count % 10 == 0)
        {
            if(this->frame_done())  //此帧完成
            {
                pthread_mutex_lock(this->output_buff_mutex);
                this->output();    //传输出去
                pthread_mutex_unlock(this->output_buff_mutex);
                
                //更新信号量
                sem_post(this->update_sem);

                this->reset();                       //重置参数
            }
        }

        this->add_count++;
    }
};

#endif
