// 1.19.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include "pch.h"
#define  _CRT_SECURE_NO_WARNINGS
#pragma warning(disable:4996)
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>
#include <vector>
#include <boost/thread/thread.hpp>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/boundary.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>  //滤波相关
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include<pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <time.h>

using namespace std;
using namespace pcl;

int PCDtoPLYconvertor(string & input_filename, string& output_filename)
{
	pcl::PCLPointCloud2 cloud;
	if (pcl::io::loadPCDFile(input_filename, cloud) < 0)
	{
		cout << "Error: cannot load the PCD file!!!" << endl;
		return -1;
	}
	PLYWriter writer;
	writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true, true);
	return 0;
}

int
main()
{
	////////////////////////////////////////获取墙体3D数据//////////////////////////////////////////////
	clock_t start, finish;
	double totaltime;
	start = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("p8.pcd", *first_cloud);
	////////////////////////////////////////采样滤波处理（下采样降低耗时）//////////////////////////////////////////////
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vg(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(first_cloud);
	vg.setLeafSize(0.5, 0.5, 0.5);
	vg.filter(*cloud_vg);
	std::cout << "PointCloud after VoxelGrid filtering has: " << cloud_vg->points.size() << " data points." << std::endl; //*
	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud(cloud);            //设置输入点云
	//pass.setFilterFieldName("z");         //设置过滤时所需要点云类型的Z字段
	//pass.setFilterLimits(0.0, 1.0);        //设置在过滤字段的范围
	//pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
	/*pass.filter(*cloud_filtered);*/
	pcl::io::savePCDFileASCII("cloud_plane_vg.pcd", *cloud_vg);
	///////////////////////////////////平面提取（采样一致性分割平面）///////////////////////////////////////////////////
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	//创建分割对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	//可选设置
	seg.setOptimizeCoefficients(true);
	//必须设置
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(5);  
	seg.setInputCloud(cloud_vg);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return (-1);
	}
	
	std::cout << "墙面方程:" << coefficients->values[0] << "x + " << coefficients->values[1] << "y + " << coefficients->values[2] << "z + "
		<< coefficients->values[3] << " = 0" << std::endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_vg);
	extract.setIndices(inliers);
	extract.setNegative(false);
	// Write the planar inliers to disk
	extract.filter(*cloud_plane);
	//std::cout << "save plane pointcloud:";
	pcl::io::savePCDFileASCII("seg_plane.pcd", *cloud_plane);
	///////////////////////////////////获取3D点云的边界///////////////////////////////////////////////////
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud = cloud_plane;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	int k = 2;
	float everagedistance = 0;
	for (int i = 0; i < cloud->size() / 2; i++)
	{
		//std::cout << "cloud->size()/2" << cloud->points[i] << std::endl;
		vector<int> nnh;
		vector<float> squaredistance;
		//  pcl::PointXYZ p;
		//   p = cloud->points[i];
		kdtree.nearestKSearch(cloud->points[i], k, nnh, squaredistance);
		/*	std::cout << "查询点位： " << cloud->points[i] << std::endl;
			std::cout << "近邻为： " << nnh[0] << "  " << nnh[1] << std::endl;
			std::cout << "近邻为： " << cloud->points[nnh[0]] << "  " << cloud->points[nnh[1]] << std::endl;*/

		everagedistance += sqrt(squaredistance[1]);
		//   cout<<everagedistance<<endl;

	}
	everagedistance = everagedistance / (cloud->size() / 2);
	cout << "everage distance is :" << everagedistance << endl;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
	normEst.setInputCloud(cloud);
	normEst.setSearchMethod(tree);
	//normEst.setRadiusSearch(2);  //法向估计的半径
	normEst.setKSearch(9);  //法向估计的点数
	normEst.compute(*normals);
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	cout << "\n计算法向量的运行时间为" << totaltime << "秒" << endl;
	//cout << "normal size is " << normals->size() << endl;
	//边界评估需要点云 
	est.setInputCloud(cloud); 
	est.setInputNormals(normals);/*M_PI_2 */
	est.setAngleThreshold(M_PI_2);   ///在这里 由于构造函数已经对其进行了初始化 为Π/2 ，必须这样 使用 M_PI/2  M_PI_2  
	est.setSearchMethod(tree);
	est.setKSearch(100);  //一般这里的数值越高，最终边界识别的精度越好 20+

	//  est.setRadiusSearch(everagedistance);  //搜索半径
	est.compute(boundaries);

	//  pcl::PointCloud<pcl::PointXYZ> boundPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
	int countBoundaries = 0;
	for (int i = 0; i < cloud->size(); i++) {
		uint8_t x = (boundaries.points[i].boundary_point);
		int a = static_cast<int>(x); //该函数的功能是强制类型转换
		if (a == 1)
		{
			//  boundPoints.push_back(cloud->points[i]);
			(*boundPoints).push_back(cloud->points[i]);
			countBoundaries++;
		}
		else
			noBoundPoints.push_back(cloud->points[i]);

	}
	std::cout << "boudary size is：" << countBoundaries << std::endl;
	//  pcl::io::savePCDFileASCII("boudary.pcd",boundPoints);
	pcl::io::savePCDFileASCII("boudary212.pcd", *boundPoints);
	pcl::io::savePCDFileASCII("NoBoundpoints.pcd", noBoundPoints);
	////////////////////////////////////////剔除离群点//////////////////////////////////////////////
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints_fliter(new pcl::PointCloud<PointXY>);
	pcl::StatisticalOutlierRemoval<pcl::PointXY> sor;
	sor.setInputCloud(boundPoints);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*boundPoints_fliter);*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints_fliter(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(boundPoints);
	sor.setRadiusSearch(2);
	sor.setMinNeighborsInRadius(5); 
	sor.setNegative(false);
	sor.filter(*boundPoints_fliter);
	std::cout << "boundPoints_fliter size is:"<< boundPoints_fliter->size()<<endl;
	pcl::io::savePCDFileASCII("boudary212_fliter.pcd", *boundPoints_fliter);
	////////////////////////////////////////边界分割//////////////////////////////////////////////
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_seg(new pcl::search::KdTree<pcl::PointXYZ>());
	//pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints_fliter_seg(new pcl::PointCloud<PointXYZ>);
	std::vector<pcl::PointIndices> boundPoints_fliter_seg_indices;
	tree_seg->setInputCloud(boundPoints_fliter);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(10);
	ec.setMaxClusterSize(2500);
	ec.setMinClusterSize(50);
	ec.setSearchMethod(tree_seg);
	ec.setInputCloud(boundPoints_fliter);
	ec.extract(boundPoints_fliter_seg_indices);
	int j = 0;  //循环新形式
	for (std::vector<pcl::PointIndices>::const_iterator it = boundPoints_fliter_seg_indices.begin(); it != boundPoints_fliter_seg_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints_fliter_seg(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
		boundPoints_fliter_seg->points.push_back(boundPoints_fliter->points[*pit]);
		boundPoints_fliter_seg->width = boundPoints_fliter_seg->points.size();
		boundPoints_fliter_seg->height = 1;
		boundPoints_fliter_seg->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << boundPoints_fliter_seg->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "boundPoints_fliter_seg_" << j << ".pcd";
		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZ>(ss.str(), *boundPoints_fliter_seg, false); 
		j++;
	}
	////////////////////////////////////////获取边界参数//////////////////////////////////////////////
	//筛选孔洞点云的思路：1.点云size。2.获取边界点云的在墙平面的投影，依据面积。3.获取体积在墙平面的投影。
	pcl::PointCloud<pcl::PointXYZ>::Ptr round_counter_3D(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("boundPoints_fliter_seg_1.pcd", *round_counter_3D);
	//创建存储点云质心的对象
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*round_counter_3D, centroid);
	std::cout << "孔洞定位坐标（"
		<< centroid[0] << ","
		<< centroid[1] << ","
		<< centroid[2] << ")." << std::endl;
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*round_counter_3D, minPt, maxPt);
	cout << "min.x = " << minPt.x << "  " << "min.y = " << minPt.y << "  "<< "min.z = " << minPt.z << endl;
	cout << "max.x = " << maxPt.x<< "  " << "max.y = " << maxPt.y << "  " << "max.z = " << maxPt.z << endl;
	cout << "BOX内x方向尺寸：" << maxPt.x - minPt.x << endl;
	cout << "BOX内y方向尺寸：" << maxPt.y - minPt.y << endl;
	cout << "BOX内中心坐标：（" << (maxPt.x - minPt.x)/2+ minPt.x << "," << (maxPt.y - minPt.y) / 2 + minPt.y<<","<< (maxPt.z - minPt.z) / 2 + minPt.z<< ")." <<endl;
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	cout << "\n此程序的运行时间为" << totaltime << "秒" << endl;
	system("pause");
}
