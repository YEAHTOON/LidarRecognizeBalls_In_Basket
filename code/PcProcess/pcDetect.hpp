#ifndef __PC_DETECT_HPP
#define __PC_DETECT_HPP

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcFilter.hpp"
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
#include <pcl/segmentation/extract_clusters.h>
// #include "pclModel.hpp"

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
        seg.setDistanceThreshold(0.005);
        seg.setProbability(0.99);
        seg.setRadiusLimits(0.095, 0.1);
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
        seg.setNormalDistanceWeight(0.5f);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.02);
        // seg.setRadiusLimits(0.0f, 1.0f);
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
        if(thePC->points.size() < source_size * 0.1) break;
    }
}

//从点云中分割聚类并获得每个聚类的质心
void getHighestPointsFromPCs(pcl::PointCloud<pcl::PointXYZ>::Ptr thePC, pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePC, pcl::PointCloud<pcl::PointXYZ>::Ptr hps)
{
    //聚类分割
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(thePC);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1f);  // 2cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (200);
    ec.setSearchMethod (tree);
    ec.setInputCloud (thePC);
    ec.extract (cluster_indices);

    //聚类的个数
    int clusters_size = cluster_indices.size();

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

        //距离滤波
        pcl::PassThrough<pcl::PointXYZ> pass;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> tempPC(new pcl::PointCloud<pcl::PointXYZ>); 
        distance_filter df(sourcePC, &pass, 0, 0, 0);
        df.realize('x', theCentroid.x + 0.15f, theCentroid.x - 0.15f, tempPC);
        df.reset('y', theCentroid.y + 0.15f, theCentroid.y - 0.15f, &pass, tempPC);
        df.realize();
        df.realize('z', 0.7f, 0.1f);

        //去除离群点
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
        outlier_filter outf(tempPC, &sor, 0.1f, 50);
        outf.realize();

        //分割球
        pcl::ModelCoefficients::Ptr sphere_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr sphere_inliers(new pcl::PointIndices);
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> tempBallPC(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ highestP; highestP.x = 0; highestP.y = 0; highestP.z = 0;
        while(1)
        {
            detector::getSphere(sphere_coefficients, sphere_inliers, tempPC);
            if(sphere_inliers->indices.size() < 1) break;     
            detector::getPCfromIndices(sphere_inliers, tempBallPC, tempPC, tempPC);

            if(sphere_coefficients->values[2] > highestP.z)
            {
                highestP.x = sphere_coefficients->values[0];
                highestP.y = sphere_coefficients->values[1];
                highestP.z = sphere_coefficients->values[2];
            }
            // std::cout << tempBallPC->points.size() << " " << sphere_coefficients->values[0] << " " << sphere_coefficients->values[1] << " " << sphere_coefficients->values[2] << std::endl;
        }

        //推入点
        if((highestP.x != 0) && (highestP.y != 0)) hps->points.push_back(highestP);



        // pcl::visualization::PCLVisualizer::Ptr viewer_ptr1(new pcl::visualization::PCLVisualizer ("ok"));
        // viewer_ptr1->addPointCloud<pcl::PointXYZ> (tempPC, "ok");
        // viewer_ptr1->spin();




        // centroids->points.push_back(theCentroid);

        // //根据质心计算点的平均高度
        // pcl::PointXYZ afterCentroid;
        // PC_squareSpace sourceSpace(0.7f, 0.1f, theCentroid.x, theCentroid.y, 0.5f, sourcePC);
        // sourceSpace.show();
        // int pc_size = sourceSpace.get_PC_count();
        // sourceSpace.getCentroid(&afterCentroid);
        // centroids->points.push_back(afterCentroid);
    }

    
}



#endif
