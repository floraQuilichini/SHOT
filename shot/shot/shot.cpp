// shot.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//#include <pcl/filters/voxel_grid.h>


int main(int argc, const char** argv)
{
	std::string keypoints_fileName = argv[1];
	std::cout << "Reading " << keypoints_fileName << std::endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr keypoint_cloud(new pcl::PointCloud<pcl::PointNormal>);

	if (pcl::io::loadPLYFile <pcl::PointNormal>(keypoints_fileName, *keypoint_cloud) == -1)
		// load the file
	{
		PCL_ERROR("Couldn't read file");
		return (-1);
	}
	std::cout << "Loaded " << keypoint_cloud->points.size() << " points." << std::endl;

	std::string surface_fileName = argv[2];
	std::cout << "Reading " << surface_fileName << std::endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr surface_cloud(new pcl::PointCloud<pcl::PointNormal>);

	if (pcl::io::loadPLYFile <pcl::PointNormal>(surface_fileName, *surface_cloud) == -1)
		// load the file
	{
		PCL_ERROR("Couldn't read file");
		return (-1);
	}
	std::cout << "Loaded " << surface_cloud->points.size() << " points." << std::endl;

	float mesh_resolution = std::stof(argv[3]);
	
	// Get the points
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud< pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*keypoint_cloud, *keypoints);
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_pts(new pcl::PointCloud< pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*surface_cloud, *surface_pts);
	
	// Get the normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud< pcl::Normal>);
	pcl::copyPointCloud<pcl::PointNormal, pcl::Normal>(*surface_cloud, *normals);


	// Setup the SHOT features
	//typedef pcl::FPFHSignature33 ShotFeature; // Can't use this, even despite: http://docs.pointclouds.org/trunk/structpcl_1_1_f_p_f_h_signature33.html
	typedef pcl::SHOT352 ShotFeature;
	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, ShotFeature> shotEstimation;
	shotEstimation.setInputCloud(keypoints);
	shotEstimation.setSearchSurface(surface_pts);
	shotEstimation.setInputNormals(normals);

	// Use KdTree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	shotEstimation.setSearchMethod(tree);
	pcl::PointCloud<ShotFeature>::Ptr shotFeatures(new pcl::PointCloud<ShotFeature>);
	shotEstimation.setRadiusSearch (5*mesh_resolution);
	//shotEstimation.setKSearch(10);

	// Actually compute the spin images
	shotEstimation.compute(*shotFeatures);
	std::cout << "SHOT output points.size (): " << shotFeatures->points.size() << std::endl;

	// write feature histograms in text file
	std::string output_text_filename = argv[4];
	std::ofstream text_file;
	text_file.open(output_text_filename.c_str(), std::ios::out);
	text_file << shotFeatures->points.size() << std::endl;
	text_file << shotFeatures->points[0].descriptorSize() << std::endl;
	for (int v = 0; v < shotFeatures->points.size(); v++) {
		const pcl::PointNormal &pt = keypoint_cloud->points[v];
		//text_file << pt.x << " " << pt.y << " " << pt.z << " "  << pt.normal_x << " " << pt.normal_y << " " << pt.normal_z << std::endl;
		text_file << pt.x << std::endl;
		text_file << pt.y << std::endl;
		text_file << pt.z << std::endl;
		const ShotFeature& descriptor = shotFeatures->points[v];
		//text_file << descriptor << std::endl;
		for (int i = 0; i < shotFeatures->points[0].descriptorSize(); i++)
		{
			text_file << descriptor.descriptor[i] << std::endl;
		}
	}
	text_file.close();

	return 0;
}

