#ifndef __PC_DETECT_HPP
#define __PC_DETECT_HPP

#include <time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcFilter.hpp"
#include "pcCommon.hpp"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <python3.10/Python.h>
#include <python3.10/numpy/arrayobject.h>
// #include "pclModel.hpp"

#define INTERAL_TIME 0.5
#define FIRST_BASKET_COUNT 80
#define SECOND_BASKET_COUNT 80
#define THIRD_BASKET_COUNT 150

/**
 * 检测器
*/
class detector
{
private:

public:

    //获取法向量
    static void getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::Normal>::Ptr output)
    {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        ne.setInputCloud(input);
        ne.setKSearch(5);
        ne.compute(*output);
    }

    //分割球
    static void getSphere(pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr thePC)
    {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_SPHERE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100000);
        seg.setDistanceThreshold(0.01);
        seg.setProbability(0.999);
        seg.setRadiusLimits(0.09, 0.1);
        seg.setInputCloud(thePC);
        seg.segment(*inliers, *coefficients);
    }

    static void getPlane(pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr thePC)
    {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.02);
        // seg.setProbability(0.99);
        seg.setInputCloud(thePC);
        seg.segment(*inliers, *coefficients);
    }

    static void getNormalPlane(pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr thePC, pcl::PointCloud<pcl::Normal>::Ptr thePC_normal)
    {
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight(0.1);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.02);
        // seg.setProbability(0.99);
        seg.setInputCloud(thePC);
        seg.setInputNormals(thePC_normal);
        seg.segment(*inliers, *coefficients);
    }

    static void getCylinder(pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr thePC, pcl::PointCloud<pcl::Normal>::Ptr thePC_normal)
    {
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setNormalDistanceWeight(0);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100000);
        seg.setDistanceThreshold(0.025);
        seg.setRadiusLimits(0.1, 0.3);
        seg.setInputCloud(thePC);
        seg.setInputNormals(thePC_normal);
        seg.segment(*inliers, *coefficients);
    }

    static void getPCfromIndices(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr targetPC, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr rest_pc, pcl::PointCloud<pcl::PointXYZ>::Ptr input)
    {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(input);
        extract.setIndices(inliers);

        if(targetPC != nullptr)
        {
            extract.setNegative(false);
            extract.filter(*targetPC);
        }

        if(rest_pc != nullptr)
        {
            extract.setNegative(true);
            extract.filter(*rest_pc);
        }
    }

    //获得质心
    static void getCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr targetPC, pcl::PointXYZ *Centroid)
    {
        int targetSize = targetPC->points.size();

        float x_gether = 0;
        float y_gether = 0;
        float z_gether = 0;

        for(int i = 0; i < targetSize; i++)
        {
            x_gether += targetPC->points[i].x;
            y_gether += targetPC->points[i].y;
            z_gether += targetPC->points[i].z;
        }

        x_gether /= targetSize;
        y_gether /= targetSize;
        z_gether /= targetSize;

        Centroid->x = x_gether;
        Centroid->y = y_gether;
        Centroid->z = z_gether;
    }

    //雷达在地面上时的拟合函数
    static void FromCentroidHeightToJudgeNUmber_OnTheGround(pcl::PointXYZ p, int *ballcount)
    {
        if(p.z == 0) {*ballcount = 0; return;}
        if(p.x * p.x + p.y * p.y > 2.56f) {*ballcount = -1; return;}
        if(p.x * p.x + p.y * p.y < 0.64f) {*ballcount = -1; return;}
        if(p.z < 0.23f) {*ballcount = -1; return;}
        if(p.z > 0.34f) {*ballcount = -1; return;}
        if(p.z < 0.255f) {*ballcount = 1; return;}
        if(p.z < 0.3f) {*ballcount = 2; return;}
        else {*ballcount = 3; return;}
    }   

    /**
     * 传入一个拟合函数与质心，判断框内球的个数
    */
    static int getBallAmout(pcl::PointXYZ p, 
    void transfun(pcl::PointXYZ, int *))
    {
        int result;
        transfun(p, &result);

        return result;
    }
};




/**
 * 一个点云空间
*/
class PC_squareSpace
{
private:
    float max_height;       //高度上限
    float min_height;       //高度下限
    float midPoint_x;       //中点的x
    float midPoint_y;       //中点的y
    float size;             //尺寸
    float max_x;
    float min_x;
    float max_y;
    float min_y;
    float max_z;
    float min_z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr spacePC;

public:

    PC_squareSpace(float maxh, float minh, float x, float y, float hs, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr target):max_height(maxh), min_height(minh), 
    midPoint_x(x), midPoint_y(y), size(hs)
    {
        this->max_z = max_height;
        this->min_z = min_height;
        this->max_x = x + this->size / 2.0f;
        this->min_x = x - this->size / 2.0f;
        this->max_y = y + this->size / 2.0f;
        this->min_y = y - this->size / 2.0f;

        //创建点云指针
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> spacePC_temp(new pcl::PointCloud<pcl::PointXYZ>); 
        this->spacePC = spacePC_temp;

        //滤波
        pcl::PassThrough<pcl::PointXYZ> pass;
        distance_filter df(target, &pass, 0, 0, 0);
        df.realize('x', this->max_x, this->min_x, this->spacePC);
        df.reset('y', this->max_y, this->min_y, &pass, this->spacePC);
        df.realize();
        df.realize('z', this->max_z, this->min_z, this->spacePC);
    }

    //获得点云数量
    int get_PC_count(void)
    {
        int result = this->spacePC->points.size();
        return result;
    }

    //获取点云质心
    void getCentroid(pcl::PointXYZ *Centroid)
    {
        detector::getCentroid(this->spacePC, Centroid);
    }

    //显示
    void show(void)
    {
        pcl::visualization::PCLVisualizer::Ptr viewer_ptr1(new pcl::visualization::PCLVisualizer ("ok"));
        viewer_ptr1->addPointCloud<pcl::PointXYZ> (this->spacePC, "ok");
        viewer_ptr1->spin();
    }
};





/**
 * 根据点云数量得到球的数量
*/
class PC_amoutJudge
{
private:
    static const int ballExist_minCount = 40; 

public:
    
    //球是否存在
    static bool BallExist(int pc_count)
    {
        if(pc_count > ballExist_minCount) return true;
        return false;
    }
};


/**
 * 分割平面
*/
void segPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr thePC)
{
    //分割平面
    pcl::ModelCoefficients::Ptr         plane_coefficients(new pcl::ModelCoefficients);     //平面的参数，无用
    pcl::PointIndices::Ptr              plane_inliers(new pcl::PointIndices);               //平面的点索引值
    int source_size = thePC->points.size();
    while(1)    //循环分割
    {
        detector::getPlane(plane_coefficients, plane_inliers, thePC);       //获得平面的点索引值
        if(plane_inliers->indices.size() < 1) break;                        //平面分割结束
        detector::getPCfromIndices(plane_inliers, nullptr, thePC, thePC);   //去除平面
        if(thePC->points.size() < source_size * 0.5) break;
    }
}

//从点云中分割聚类并获得每个聚类的质心
void getHighestPointsFromPCs(pcl::PointCloud<pcl::PointXYZ>::Ptr thePC, pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePC, pcl::PointCloud<pcl::PointXYZ>::Ptr hps, 
PyObject *model, PyObject *pred_fun)
{
    //聚类分割
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(thePC);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1f);  // 2cm
    ec.setMinClusterSize (30);
    ec.setMaxClusterSize (1000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (thePC);
    ec.extract (cluster_indices);

    //聚类的个数
    int clusters_size = cluster_indices.size();

    //显示器
// #define view
#ifdef view
    pcl::visualization::PCLVisualizer::Ptr viewer_ptr1(new pcl::visualization::PCLVisualizer ("ok"));
#endif

// #define store_points
#ifdef store_points
    //初始化pts存储器
    PCs_outer pc_storer("/home/yezhiteng/PROJECTS/PRODUCTS/ENVIRONMENT_SENSE/stored_points/three");
#endif

    //创建聚类的点云空间
    for(int i = 0; i < clusters_size; i++)
    {
        //获得每个聚类点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPC(new pcl::PointCloud<pcl::PointXYZ>);
        std::shared_ptr<pcl::PointIndices> theIndices_ptr = pcl::PointIndices::Ptr(new pcl::PointIndices);
        *theIndices_ptr = cluster_indices.at(i);
        detector::getPCfromIndices(theIndices_ptr, clusterPC, nullptr, thePC);

        //获得聚类的质心
        pcl::PointXYZ theCentroid;
        detector::getCentroid(clusterPC, &theCentroid);
        float centroid_distance = fsqrt(powf(theCentroid.x, 2.0f) + powf(theCentroid.y, 2.0f));

#ifdef store_points
        if(((theCentroid.x < 1.8f)&&(theCentroid.x > 1.6f))&&((theCentroid.y > -1.7f)&&(theCentroid.y < 1.7f)))
        {
            // 获取本地时间
            struct timeval present_time = {0};
            gettimeofday(&present_time, NULL);
            time_t temp = time(NULL);
            struct tm *lt = localtime(&temp);
            //存
            char pc_file_name[32] = {0}; sprintf(pc_file_name, "%d%d%ld_%d", lt->tm_min, lt->tm_sec, present_time.tv_usec, i);
            pc_storer.store_pc(clusterPC, pc_file_name);
        }
#endif

        #define USE_PYTHON 1
        #if USE_PYTHON

        // 以z值作为球数量的判断依据
        pcl::PointXYZ highestP; highestP.x = theCentroid.x; highestP.y = theCentroid.y; highestP.z = 0;   

        // 将点云变为numpy数组
        float clusterPC_in_constance[clusterPC->size()*3] = {0}; float *temp_pointer = clusterPC_in_constance;
        for(auto it = clusterPC->begin(); it != clusterPC->end(); ++it)
        {
            pcl::PointXYZ temp_point = *it;
            temp_pointer[0] = temp_point.x;
            temp_pointer[1] = temp_point.y;
            temp_pointer[2] = temp_point.z;
            temp_pointer = temp_pointer + 3;
        }
        npy_intp Dims[2] = {(int)clusterPC->size(), 3};
        PyObject* points_in_python   = (PyObject*)PyArray_SimpleNewFromData(2, Dims, NPY_FLOAT32, (void *)clusterPC_in_constance);    //获得python中的numpy数组

        // 调用函数
        PyObject *point_number  = PyLong_FromLong(300); 
        PyObject *pred_args     = PyTuple_Pack(3, model, points_in_python, point_number);    //传入的参数
        PyObject *result_list   = PyObject_CallObject(pred_fun, pred_args);         //调用检测
        if (PyErr_Occurred()) {
        PyErr_PrintEx(0);
        PyErr_Clear(); // this will reset the error indicator so you can run Python code again
        }
        PyObject *result_py = PyList_GetItem(result_list, 0);
        int result = PyLong_AsLong(result_py);

        // 实际意义化
        if(result == 0) highestP.z = 0.1f;
        else if(result == 1) highestP.z = 0.5f;
        else if(result == 2) highestP.z = 0.3f;
        else if(result == 3) highestP.z = 0.0f;

        #else

        // //距离滤波
        // pcl::PassThrough<pcl::PointXYZ> pass;
        // std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> tempPC(new pcl::PointCloud<pcl::PointXYZ>); 
        // distance_filter df(sourcePC, &pass, 0, 0, 0);
        // df.realize('x', theCentroid.x + 0.15f, theCentroid.x - 0.15f, tempPC);
        // df.reset('y', theCentroid.y + 0.15f, theCentroid.y - 0.15f, &pass, tempPC);
        // df.realize();
        // df.realize('z', 0.7f, 0.05f);



        // //去除离群点pcl::visualization::PCLVisualizer::Ptr viewer_ptr1(new pcl::visualization::PCLVisualizer ("ok"));
        // pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
        // outlier_filter outf(tempPC, &sor, 0.1f, 20);
        // outf.realize();

        // //分割圆柱，获得圆柱参数
        // pcl::ModelCoefficients::Ptr         cylinder_coefficients(new pcl::ModelCoefficients);
        // pcl::PointIndices::Ptr              cylinder_inliers(new pcl::PointIndices); 
        // pcl::PointCloud<pcl::Normal>::Ptr   normal_pc(new pcl::PointCloud<pcl::Normal>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_PC(new pcl::PointCloud<pcl::PointXYZ>);
        // detector::getNormals(clusterPC, normal_pc);
        // detector::getCylinder(cylinder_coefficients, cylinder_inliers, clusterPC, normal_pc);
        // detector::getPCfromIndices(cylinder_inliers, cylinder_PC, clusterPC, clusterPC);
        // float cylinder_x = cylinder_coefficients->values[0];
        // float cylinder_y = cylinder_coefficients->values[1];
        // float cylinder_r = cylinder_coefficients->values[6];
        // float cylinder_angle = acosf(fabs(cylinder_coefficients->values[5]));
        // // std::cout << cylinder_coefficients->values[5] << " " << cylinder_angle << std::endl;
        // if((cylinder_r > 0.13)||(cylinder_r < 0.1)||(cylinder_angle > 0.1)) continue;
        // // std::cout << cylinder_x << " " << cylinder_y << " " << cylinder_r << std::endl;

        // //滤掉圆柱点
        // pcl::PointCloud<pcl::PointXYZ>::Ptr balls_PC(new pcl::PointCloud<pcl::PointXYZ>);
        // for(auto it = clusterPC->begin(); it != clusterPC->end(); ++it)
        // {
        //     pcl::PointXYZ temp = *it;

        //     float distance2 = (temp.x-cylinder_x)*(temp.x-cylinder_x) + (temp.y-cylinder_y)*(temp.y-cylinder_y);
        //     float tolerance = -0.015f;
        //     if(distance2 < (cylinder_r+tolerance)*(cylinder_r+tolerance)) balls_PC->push_back(temp);
        // }

        //去除离群点
        float distance_K = fsqrt(theCentroid.x*theCentroid.x + theCentroid.y*theCentroid.y);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
        outlier_filter outf(clusterPC, &sor, 0.2f, 30);
        // outf.realize(centroid_distance, [](float d)
        // {
        //     float p1[2] = {1.65f, INTERAL_TIME * 500}, p2[2] = {1.6f * 1.44f, INTERAL_TIME * 2000};
        //     float K = (p1[1] - p2[1]) / (p1[0] - p2[0]);
        //     float offset = (p1[1]*p2[0] - p2[1]*p1[0]) / (p2[0] - p1[0]);
        //     int result = (int)(K * d + offset);
        //     return result;
        // });

        // std::cout << cylinder_PC->size() << std::endl;

        //添加显示点云
#ifdef view
        char a[20] = {0};
        sprintf(a, "%d", i);
        viewer_ptr1->addPointCloud<pcl::PointXYZ> (clusterPC, a);
#endif

        //分割球
        pcl::ModelCoefficients::Ptr sphere_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr sphere_inliers(new pcl::PointIndices);
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> tempBallPC(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ highestP; highestP.x = 0; highestP.y = 0; highestP.z = 0;
        while(1)
        {
            //执行分割球
            detector::getSphere(sphere_coefficients, sphere_inliers, clusterPC);
            if(sphere_inliers->indices.size() < 5) break;     
            detector::getPCfromIndices(sphere_inliers, tempBallPC, clusterPC, clusterPC);
            // if(tempBallPC->size() < 10) continue; 

            // //后验
            // //球心位置判断
            // float sphere_x = sphere_coefficients->values[0];
            // float sphere_y = sphere_coefficients->values[1];
            // float distance_MM = fsqrt((sphere_x-cylinder_x)*(sphere_x-cylinder_x) + (sphere_y-cylinder_y)*(sphere_y-cylinder_y));
            // if(distance_MM > 0.04f) continue; 
            // //球内点云分布判断


            if(sphere_coefficients->values[2] > highestP.z)
            {
                highestP.x = sphere_coefficients->values[0];
                highestP.y = sphere_coefficients->values[1];
                highestP.z = sphere_coefficients->values[2];
            }
        }

        #endif

        //推入点
        if((highestP.x != 0) && (highestP.y != 0)) hps->points.push_back(highestP);
    }

#ifdef view
    viewer_ptr1->spin();
#endif
    
}



#endif
