#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
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
#include "./code/tcp/tcp.hpp"
#include "./code/PcProcess/pcFilter.hpp"
#include "./code/PcProcess/pcCommon.hpp"
#include <pthread.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <math.h>
#include <vector>
#include <pcl/segmentation/extract_clusters.h>
#include <semaphore.h>
#include <fstream>
#include "log.hpp"
#include <python3.10/Python.h>


float offset[3] = {0};      // 动态矫正点坐标
float base[3] = {0};
float robot_position[2] = {0};
pcl::PointCloud<pcl::PointXYZ>::Ptr buff;   //声明缓冲区及其锁
pthread_mutex_t buff_mutex, log_mutex;
sem_t updatePC;         //声明信号量
yzt_log *lfp;
void close_gently(int signum);



/**
 * 回调更新点云
*/
// PointCloud_Frame pointCloud_Frame(100000, 100000, buff, &buff_mutex);
PointCloud_Frame pointCloud_Frame(10000000, INTERAL_TIME * 1000000);
void pc_callback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
{
    //点云添加
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
    pointCloud_Frame.updating(p_point_data, data->dot_num, offset);
    // std::cout << "ok" << std::endl;
}

/**
 * 更新偏移量
*/
void *process_masterInfo(void *args)
{
    while(1);
}

//显示点云
void showPC(pcl::PointCloud<pcl::PointXYZ>::Ptr thePC, const char *name)
{
    pcl::visualization::PCLVisualizer::Ptr viewer_ptr1(new pcl::visualization::PCLVisualizer (name));
    viewer_ptr1->addPointCloud<pcl::PointXYZ> (thePC, name);
    viewer_ptr1->spin();
}

/**
 * 接收主控消息
*/
typedef struct 
{
    uint8_t send_flag = 0;
    TCP_CommunicationManager *m;
} respond_request_Args;
void *respond_request(void *args_void)          //响应请求
{
    //初始化
    respond_request_Args *args = (respond_request_Args *)args_void;
    TCP_CommunicationManager *tcp_m = args->m;
    uint8_t request[128] = {0};

    while(1)
    {
        //接收信号
        tcp_m->Recv(request, 128);
        if(strlen((char *)request) == 0)
        {
            close_gently(-1);
        }

        //打印接收到的消息
        // *lfp << "get a request";
        // *lfp << (char *)request;

        //主控使能
        if((request[0] == 'M')&&(request[1] == 'O'))
        {
            args->send_flag = 1;
        }
        //主控失能
        else if((request[0] == 'S')&&(request[1] == 'T'))
        {
            args->send_flag = 0;
        }
        //车坐标
        else if((request[0] == 'A')&&(request[1] == 'L'))
        {
            robot_position[0] = (float)(*((int16_t *)(&request[2]))) / 1000.0f;
            robot_position[1] = (float)(*((int16_t *)(&request[4]))) / 1000.0f;
            offset[0] = robot_position[0] - base[0];
            offset[1] = robot_position[1] - base[1];
            // char x[128] = {0};
            // char y[128] = {0};
            // sprintf(x, "from shenchang : %f ", robot_position[0]);
            // sprintf(y, "from shenchang : %f ", robot_position[1]);
            // *lfp << x;
            // *lfp << y;
        }

        //重置
        memset(request, 0, 128);
        usleep(10000);
    }
}


//优雅地关闭程序
int CloseSocketNum = -1;
void close_gently(int signum)
{
    Py_Finalize();
    if(CloseSocketNum >= 0) close(CloseSocketNum);
    *lfp << "close gently";
    LivoxLidarSdkUninit();
    exit(1);
}


//检测Python错误
void python_check_error()
{
    if (PyErr_Occurred()) {
        PyErr_PrintEx(0);
        PyErr_Clear(); // this will reset the error indicator so you can run Python code again
    }
}



/**
 * 主函数
*/
int main(int argc, char **argv)
{
    //输入参数个数错误
    if(argc < 2){
        std::cout << "lidar process arguements number error" << std::endl;
        exit(-1);
    }

    //初始化缓冲区及其锁
    buff = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pthread_mutex_init(&buff_mutex, nullptr);
    pthread_mutex_init(&log_mutex, nullptr);
    pointCloud_Frame.set_outputBuff(buff, &buff_mutex);     //正式设置为缓冲区

    // 初始化log
    yzt_log log_file("lidar.txt", "LIDAR", &log_mutex);
    lfp = &log_file;

    // 初始化信号量
    sem_init(&updatePC, 0, 0);
    pointCloud_Frame.set_updatesem(&updatePC);

    // 初始化雷达SDK
    if (!LivoxLidarSdkInit(argv[1]))
    {
        printf("Livox Init Failed\n");
        LivoxLidarSdkUninit();
        close_gently(-1);
    }

#define sendinfo
#ifdef sendinfo
    // 初始化网络通信
    Addr serverAddr((char *)"127.0.0.1", 30000);                    //服务器地址
    int socketNum = socket(AF_INET, SOCK_STREAM, 0);        //获得套接字
    CloseSocketNum = socketNum;
    yzt_tcp::connectToServer(serverAddr, socketNum);        //连接
    TCP_CommunicationManager tcp_c_m(socketNum);
    const char *sure = "lidar";
    tcp_c_m.Send((void *)sure, strlen(sure));
    
    // 创建线程
    pthread_t pid[10];
    respond_request_Args respond_request_args;
    respond_request_args.m = &tcp_c_m;
    pthread_create(pid, NULL, respond_request, &respond_request_args);
#endif

    //重新注册信号
    signal(SIGINT, close_gently);
    signal(SIGTERM, close_gently);

    //配置python环境
    Py_Initialize();        //启动python解释器
    _import_array();        //初始化numpy
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("import os");
    PyRun_SimpleString("import random");
    PyRun_SimpleString("import numpy");
    PyRun_SimpleString("sys.path.append(\'/home/yezhiteng/anaconda3/envs/ROS2/lib/python3.10/site-packages\')");
    PyRun_SimpleString("sys.path.append(\'/home/yezhiteng/anaconda3/lib/python3.9/site-packages\')");
    PyRun_SimpleString("sys.path.append(\'/home/yezhiteng/INCLUDES/pointnet.pytorch-master\')");
    PyRun_SimpleString("import torch");
    PyRun_SimpleString("os.chdir(\'/home/yezhiteng/PROJECTS/PRODUCTS/ENVIRONMENT_SENSE/code\')");
    PyRun_SimpleString("sys.path.append(\'.\')");

    // 导入自定义模块
    PyObject *detect_module     = PyImport_ImportModule("detect");                      //获得模块
    PyObject *detect_fun_dict   = PyModule_GetDict(detect_module);                      //获得函数字典集

    // 获得模型
    PyObject *load_pointnet_model   = PyDict_GetItemString(detect_fun_dict, "load_pointnet_model");     //获得模型函数的句柄
    PyObject *pth_path              = PyUnicode_FromString("/home/yezhiteng/PROJECTS/PRODUCTS/ENVIRONMENT_SENSE/model_relate/202404102.pth");
    PyObject *pointnet_model        = PyObject_CallObject(load_pointnet_model, PyTuple_Pack(1, pth_path));                //调用函数，获得模型
    python_check_error();

    // 获得推理函数
    PyObject *get_pred = PyDict_GetItemString(detect_fun_dict, "get_pred");     //获得模型函数的句柄

    //启动线程，开启回调函数
    SetLivoxLidarPointCloudCallBack(pc_callback, nullptr);

    // 循环中使用到的算法类
    pcl::PassThrough<pcl::PointXYZ> pass;           //距离滤波器
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;   //离群滤波器

    while(1)
    {
        //等待更新
        sem_wait(&updatePC);
        int sem_count = 0;
        base[0] = robot_position[0];
        base[1] = robot_position[1];
        sem_getvalue(&updatePC, &sem_count);
        if(sem_count > 0){      //直到信号量复位为0
            sem_wait(&updatePC);
        }

        // //记录检测坐标
        // char base_x_log[128] = {0};
        // char base_y_log[128] = {0};
        // sprintf(base_x_log, "%f", base[0]);
        // sprintf(base_y_log, "%f", base[1]);
        // *lfp << base_x_log;
        // *lfp << base_y_log;

        //获得点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr thePC(new pcl::PointCloud<pcl::PointXYZ>);
        pthread_mutex_lock(&buff_mutex);
        copyPointCloud(*buff, *thePC);
        pthread_mutex_unlock(&buff_mutex);
        // showPC(thePC, "name");

        //复制一份原点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePC(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(*thePC, *sourcePC);

        //距离滤波
        distance_filter df(thePC, &pass, 0, 0, 0);
        df.realize('x', 2.0f, 0.2f);
        df.realize('y', 5.0f, -5.0f);
        df.realize('z', 0.8f, 0.05f);
        // showPC(thePC, "name");

        // 去除离群点
        outlier_filter outf(thePC, &sor, 0.2f, INTERAL_TIME * 100);
        outf.realize();

        //分割平面
        // segPlane(thePC);
        // showPC(thePC, "name");

        //再次去除离群点
        outf.realize();

        //获取球心
        pcl::PointCloud<pcl::PointXYZ>::Ptr HighestPoints(new pcl::PointCloud<pcl::PointXYZ>);
        getHighestPointsFromPCs(thePC, sourcePC, HighestPoints, pointnet_model, get_pred);

        //目标数
        int target_nums = HighestPoints->points.size();

        //创建tcp数据包
        int tcp_pack_len = target_nums * 5 + 5;
        uint8_t dataToSend[tcp_pack_len + 1];
        dataToSend[0] = tcp_pack_len;       //报告给tcp的总长
        dataToSend[1] = 0x5a;
        dataToSend[2] = 0xee;
        dataToSend[3] = (uint8_t)(target_nums * 5);
        dataToSend[tcp_pack_len - 1] = 0xff;
        dataToSend[tcp_pack_len] = 0xfe;
        
        //根据点的个数获得点的个数
        for(int i = 0; i < target_nums; i++)
        {
            //高度变成数目
            int ballcount = 0;
            if(HighestPoints->points[i].z > 0.42f) ballcount = 3;
            else if(HighestPoints->points[i].z > 0.23f) ballcount = 2;
            else if(HighestPoints->points[i].z > 0.05f) ballcount = 1;

            //存入tcp数据包
            memcpy(dataToSend+4+5*i, &HighestPoints->points[i].y, 4);
            dataToSend[8+5*i] = ballcount;

            // printf("ball : %f %f %f\n", HighestPoints->points[i].x, HighestPoints->points[i].y, HighestPoints->points[i].z);
            // printf(" x轴%-4f, y轴%-4f 处, 有 %d 个球。\n", HighestPoints->points[i].x, HighestPoints->points[i].y, ballcount);
            char log_char[128] = {0};
            sprintf(log_char, " x轴%-4f, y轴%-4f, z轴%-4f 处， 有 %d 个球。",  
            HighestPoints->points[i].x, HighestPoints->points[i].y, HighestPoints->points[i].z, ballcount);
            *lfp << log_char;
        }

#ifdef sendinfo
        //tcp发送信息
        if((target_nums != 0)&&(respond_request_args.send_flag == 1)) 
        {
            tcp_c_m.Send(dataToSend, tcp_pack_len + 1);
            *lfp << "send data";
            *lfp << (char *)dataToSend;
        }
#endif
    }

    //关闭SDK
    LivoxLidarSdkUninit();
    while(1);
}