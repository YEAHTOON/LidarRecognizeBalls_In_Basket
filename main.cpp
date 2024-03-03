#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
#include <unistd.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include "./code/livoxToPc/getPC.hpp"
#include "./code/PcProcess/pcDetect.hpp"
#include "./code/PcProcess/pcFilter.hpp"
#include <pthread.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <math.h>
#include <vector>
#include <pcl/segmentation/extract_clusters.h>
#include <semaphore.h>
// #include <pcl/segmentation/extract_clusters.h>

//声明缓冲区及其锁
pcl::PointCloud<pcl::PointXYZ>::Ptr buff;
pthread_mutex_t buff_mutex;

//声明信号量
sem_t updatePC;

/**
 * 回调更新点云
*/
// PointCloud_Frame pointCloud_Frame(100000, 100000, buff, &buff_mutex);
PointCloud_Frame pointCloud_Frame(10000000, 250000);
void pc_callback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
{
    //点云添加
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
    pointCloud_Frame.updating(p_point_data, data->dot_num);
    // std::cout << "ok" << std::endl;
}



//显示点云
void showPC(pcl::PointCloud<pcl::PointXYZ>::Ptr thePC, const char *name)
{
    pcl::visualization::PCLVisualizer::Ptr viewer_ptr1(new pcl::visualization::PCLVisualizer (name));
    viewer_ptr1->addPointCloud<pcl::PointXYZ> (thePC, name);
    viewer_ptr1->spin();
}



/**
 * 主函数
*/
int main(void)
{
    //初始化缓冲区及其锁
    buff = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pthread_mutex_init(&buff_mutex, nullptr);
    pointCloud_Frame.set_outputBuff(buff, &buff_mutex);     //正式设置为缓冲区

    //初始化信号量
    sem_init(&updatePC, 0, 0);
    pointCloud_Frame.set_updatesem(&updatePC);

    //初始化雷达SDK
    if (!LivoxLidarSdkInit("../config.json")) 
    {
        printf("Livox Init Failed\n");
        LivoxLidarSdkUninit();
        return -1;
    }

    //启动线程，开启回调函数
    SetLivoxLidarPointCloudCallBack(pc_callback, nullptr);

    // //等待线程
    // sleep(2);

    while(1)
    {
        //等待更新
        sem_wait(&updatePC);
        int sem_count = 0;
        sem_getvalue(&updatePC, &sem_count);
        if(sem_count > 0)       //直到信号量复位为0
        {
            sem_wait(&updatePC);
        }

        //获得点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr thePC(new pcl::PointCloud<pcl::PointXYZ>);
        pthread_mutex_lock(&buff_mutex);
        copyPointCloud(*buff, *thePC);
        pthread_mutex_unlock(&buff_mutex);

        //复制一份原点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePC(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(*thePC, *sourcePC);

        //距离滤波
        pcl::PassThrough<pcl::PointXYZ> pass;
        distance_filter df(thePC, &pass, 0, 0, 0);
        df.realize('x', 5.0f, 0.0f);
        df.realize('y', 5.0f, -5.0f);
        df.realize('z', 0.5f, 0.0f);

        //去除离群点
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
        outlier_filter outf(thePC, &sor, 0.1f, 50);
        outf.realize();

        //分割平面
        segPlane(thePC);

        //再次去除离群点
        outf.realize();

        //获取质心
        pcl::PointCloud<pcl::PointXYZ>::Ptr HighestPoints(new pcl::PointCloud<pcl::PointXYZ>);
        getHighestPointsFromPCs(thePC, sourcePC, HighestPoints);

        for(int i = 0; i < HighestPoints->points.size(); i++)
        {
            int ballcount = 0;
            if(HighestPoints->points[i].z > 0.45f) ballcount = 3;
            else if(HighestPoints->points[i].z > 0.27f) ballcount = 2;
            else if(HighestPoints->points[i].z > 0.1f) ballcount = 1;
            printf(" x轴%-4f, y轴%-4f 处, 有 %d 个球。\n", HighestPoints->points[i].x, HighestPoints->points[i].y, ballcount);

            // printf("%-9f %-9f %-9f\n", HighestPoints->points[i].x, HighestPoints->points[i].y, HighestPoints->points[i].z);
            // int ballcount = detector::getBallAmout(HighestPoints->points[i], detector::FromCentroidHeightToJudgeNUmber_OnTheGround);
            // if(ballcount != -1) printf("%-9f %-9f  :  %-d\n", HighestPoints->points[i].x, HighestPoints->points[i].y, ballcount);
            // printf("%-9f %-9f %-9f\n", HighestPoints->points[i].x, HighestPoints->points[i].y, HighestPoints->points[i].z - 0.1f);
        }

        printf("-----------\n");
    }

    // int result1_ballcount = detector::getBallAmout(HighestPoints->points[0], detector::FromCentroidHeightToJudgeNUmber_OnTheGround);
    // int result2_ballcount = detector::getBallAmout(HighestPoints->points[1], detector::FromCentroidHeightToJudgeNUmber_OnTheGround);
    // int result3_ballcount = detector::getBallAmout(HighestPoints->points[2], detector::FromCentroidHeightToJudgeNUmber_OnTheGround);

    // std::cout << "\nball count:" << std::endl;
    // std::cout << result1_ballcount << std::endl;
    // std::cout << result2_ballcount << std::endl;
    // std::cout << result3_ballcount << std::endl;



    // PC_squareSpace result1_inner(0.5f, 0.1f, result1_centroid.x, result1_centroid.y, 0.05f);
    // PC_squareSpace result1_outer(0.5f, 0.1f, result1_centroid.x, result1_centroid.y, 0.2f);
    // std::cout << result1_inner.get_PC_count(sourcePC) << " " << result1_outer.get_PC_count(sourcePC) << std::endl;

    // PC_squareSpace result2_inner(0.5f, 0.1f, result2_centroid.x, result2_centroid.y, 0.05f);
    // PC_squareSpace result2_outer(0.5f, 0.1f, result2_centroid.x, result2_centroid.y, 0.2f);
    // std::cout << result2_inner.get_PC_count(sourcePC) << " " << result2_outer.get_PC_count(sourcePC) << std::endl;

    // PC_squareSpace result3_inner(0.5f, 0.1f, result3_centroid.x, result3_centroid.y, 0.05f);
    // PC_squareSpace result3_outer(0.5f, 0.1f, result3_centroid.x, result3_centroid.y, 0.2f);
    // std::cout << result3_inner.get_PC_count(sourcePC) << " " << result3_outer.get_PC_count(sourcePC) << std::endl;





    


    // // int noPlaneSize = thePC->points.size();
    // // while(1)
    // // {
    //     detector::getCylinder(plane_coefficients, plane_inliers, thePC, thePC_normal);       //获得平面的点索引值
    //     // if(plane_inliers->indices.size() < 1) break;                        //平面分割结束
    //     detector::getPCfromIndices(plane_inliers, plane_pc, thePC, thePC);   //去除平面
    //     // if(thePC->points.size() < noPlaneSize * 0.3) break;
    //     detector::getNormals(thePC, thePC_normal);
    //     std::cout << "success1" << std::endl;
    // // }

    // 滤波
    // PC_squareSpace basket(500.0f, 100.0f, 800.0f, 0.0f, 150.0f);
    // int pc_count = basket.get_PC_count(thePC);
    // pc_count = pc_count - THIRD_BASKET_COUNT;
    // pc_count = ;     // 第二个箩筐的参数
    // pc_count = 3;
    // std::cout << pc_count << std::endl;
    // pcl::PassThrough<pcl::PointXYZ> pass;
    // distance_filter df(thePC, &pass, 0, 0, 0);
    // df.realize('x', 900.0f, 700.0f);
    // df.realize('y', 100.0f, -100.0f);
    // df.realize('z', 500.0f, 100.0f);
    // pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
    // outlier_filter outf(thePC, &sor, 190.0f, 100);
    // outf.realize();
    // int pc_count = thePC->points.size();

    // int three_count = 200;
    // int two_count = 110;
    // int one_count = 50;
    // int zero_count = 0;

    // const int three_close = abs(pc_count - three_count);
    // const int two_close = abs(pc_count - two_count);
    // const int one_close = abs(pc_count - one_count);
    // const int zero_close = abs(pc_count - zero_count);

    // int close[4] = {zero_close, one_close, two_close, three_close};
    // for(int i = 0; i < 4; i++)
    // {
    //     for(int j = 0; j < (3-i); j++)
    //     {
    //         if(close[j] > close[j + 1])
    //         {
    //             int temp = close[j];
    //             close[j] = close[j+1];
    //             close[j+1] = temp;
    //         }
    //     }
    // }

    // std::cout << close[0] << " " << close[1] << " " << close[2] << " " << close[3] << std::endl;

    // if(zero_close == close[0])
    // {
    //     std::cout << "\n一共有0个球\n" << std::endl;
    // }
    // else if(one_close == close[0])
    // {
    //     std::cout << "\n一共有1个球\n" << std::endl;
    // }
    // else if(two_close == close[0])
    // {
    //     std::cout << "\n一共有2个球\n" << std::endl;
    // }
    // else if(three_close == close[0])
    // {
    //     std::cout << "\n一共有3个球\n" << std::endl;
    // }
    
    
    //显示对象
    // pcl::visualization::PCLVisualizer::Ptr viewer_ptr1(new pcl::visualization::PCLVisualizer ("visual"));
    // // 球的参数与球的点云索引
    // pcl::ModelCoefficients::Ptr sphere_coefficients(new pcl::ModelCoefficients);    //球的参数
    // pcl::PointIndices::Ptr sphere_inliers(new pcl::PointIndices);                   //球点云的索引值
    // pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_pc(new pcl::PointCloud<pcl::PointXYZ>);
    // int sphere_count = 0;
    // while(1)
    // {
    //     //寻找球
    //     detector::getSphere(sphere_coefficients, sphere_inliers, thePC);        //使用算法寻找到球
    //     if(sphere_inliers->indices.size() < 1) break;                           //无法寻找到球，退出循环
    //     detector::getPCfromIndices(sphere_inliers, sphere_pc, thePC, thePC);

    //     std::cout << sphere_coefficients->values[0] << " " << sphere_coefficients->values[1] << " " << sphere_coefficients->values[2] << std::endl;
    //     std::cout << sphere_coefficients->values[3] << " " << sphere_inliers->indices.size() << std::endl;

    //     char a[2];
    //     sprintf(a, "%d", sphere_count);

    //     pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb(sphere_pc, "z");
    //     viewer_ptr1->addPointCloud(sphere_pc, rgb, a);

    //     sphere_count++;
    // }
    // viewer_ptr1->addPointCloud<pcl::PointXYZ>(thePC, "rest");
    // viewer_ptr1->spin();

    //计时初始时刻
    // struct timeval p_present_time = {0};
    // gettimeofday(&p_present_time, NULL);

    //计时结束时刻
    // struct timeval a_present_time = {0};
    // gettimeofday(&a_present_time, NULL);
    // std::cout << a_present_time.tv_usec - p_present_time.tv_usec << std::endl;

    // pcl::visualization::PCLVisualizer::Ptr viewer_ptr1(new pcl::visualization::PCLVisualizer ("fine"));
    // // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb(plane_pc, "z");
    // viewer_ptr1->addPointCloud<pcl::PointXYZ> (result1, "result1");
    // viewer_ptr1->addPointCloud<pcl::PointXYZ> (result2, "result2");
    // viewer_ptr1->addPointCloud<pcl::PointXYZ> (result3, "result3");
    // // viewer_ptr1->addPointCloud(plane_pc, rgb, "sample");
    // // viewer_ptr1->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (thePC, thePC_normal, 10, 200.0f, "normals");
    // viewer_ptr1->spin();


    // showColorPC(thePC, rgb, "color");
    // showPC(thePC, "rest");

    //关闭SDK
    LivoxLidarSdkUninit();
    while(1);
}
