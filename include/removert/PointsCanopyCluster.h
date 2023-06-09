//
// Created by zhuhongwei on 2023/4/1.
//

#include <iostream>
#include <vector>
#include <cmath>
#include <removert/utility.h>
#include <ctime>
#include <cstdlib>
//#include "PointsK-means.h"


float euclidean_distance(const PointType& p1, const PointType& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

/**
 * Not implemented with PCL
 */
std::vector<std::vector<PointType>> canopy_clustering(const std::vector<PointType, Eigen::aligned_allocator<PointType> >& points, float t1, float t2) {

    std::vector<PointType, Eigen::aligned_allocator<PointType> > unassigned_points = points;
    std::vector<std::vector<PointType>> clusters;

    while (!unassigned_points.empty()) {
        PointType seed_point = unassigned_points.back();
        unassigned_points.pop_back();

        std::vector<PointType> cluster;
        cluster.push_back(seed_point);

        for (auto it = unassigned_points.begin(); it != unassigned_points.end();) {
            float distance = euclidean_distance(seed_point, *it);
            if (distance < t1) {
                cluster.push_back(*it);
                it = unassigned_points.erase(it);
            } else {
                ++it;
            }
        }

        for (auto it = cluster.begin(); it != cluster.end();) {
            float distance = euclidean_distance(seed_point, *it);
            if (distance < t2) {
                ++it;
            } else {
                unassigned_points.push_back(*it);
                it = cluster.erase(it);
            }
        }

        clusters.push_back(cluster);
    }

    return clusters;
}

void VectorPoints2PointPtr(std::vector<std::vector<PointType>> clusters, pcl::PointCloud<PointType>::Ptr& PointsPtr) {
    for (int i = 0; i < clusters.size(); ++i) {
        pcl::PointCloud<PointType>::Ptr NewPtr(new pcl::PointCloud<PointType>);
        NewPtr->insert(NewPtr->end(), clusters[i].begin(), clusters[i].end());
        *PointsPtr += *NewPtr;
    }
}

/**
 * Canopy algorithm implemented using pcl
 * @param PointsPtr
 * @param CanopyRadius
 * @param Canopyt1
 * @param Canopyt2
 */
void CanopyClusteringPCL(pcl::PointCloud<PointType>::Ptr PointsPtr, float CanopyRadius, float Canopyt1, float Canopyt2) {

    pcl::KdTreeFLANN<PointType > kdtree;
    kdtree.setInputCloud(PointsPtr);
    std::vector<std::vector<int>> clusters;
    std::vector<bool> visited(PointsPtr->size(),false);
    for(int i = 0 ; i <PointsPtr->size(); ++i)
    {
        if (visited[i]) continue;
        std::vector<int> CanopyCluster;
        std::vector<int> neighbors;
        std::vector<float> distances;
        kdtree.radiusSearch(i,CanopyRadius,neighbors, distances );
        for(int j = 0 ; j< neighbors.size(); ++j)
        {
            if (distances[j] < Canopyt1){
                CanopyCluster.push_back(neighbors[j]);
                visited[neighbors[j]] = true;
            }
        }
        if(CanopyCluster.empty()) continue;

        for(int j = 0 ; j< neighbors.size(); ++j)
        {
            if (distances[j] < Canopyt2){
                visited[neighbors[j]] = true;
            } else{
                CanopyCluster.erase(std::remove(CanopyCluster.begin(),CanopyCluster.end(), neighbors[j]),CanopyCluster.end());
                visited[neighbors[j]] = false;
            }
        }
        clusters.push_back(CanopyCluster);
    }
}

void PointsVisual(std::vector<std::vector<PointType>> clusters, pcl::PointCloud<PointType>::Ptr PointsPtr) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<std::vector<int> > init = {{255,0,0},{0,255,0},{0,0,255},{0,0,0}};
    for (int i = 0; i < clusters.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr NewPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
        srand(time(nullptr));
        int RandomNum = rand() % 5;
        std::vector<int> v_color = init[RandomNum];
        for (int j = 0; j < clusters[i].size(); ++j) {
            pcl::PointXYZRGB p;
            p.x = clusters[i][j].x;
            p.y = clusters[i][j].y;
            p.z = clusters[i][j].z;
            p.r = v_color[0];
            p.g = v_color[1];
            p.b = v_color[2];
            NewPtr->push_back(p);
        }
        *ColoredCloud += *NewPtr;
    }
    ROS_INFO_STREAM("\033[1;32m -- Save the ColoredCloud "  "\033[0m");
    pcl::io::savePCDFile<pcl::PointXYZRGB>(  "/home/zhuhongwei/Removert_sci4/src/removert/PCDData/KITTY/test08_2/outputcloud.pcd",*ColoredCloud);
}


void MainProcessCanopy( pcl::PointCloud<PointType>::Ptr DynamicPointsPtr, float t1, float t2 ){
    std::vector<PointType, Eigen::aligned_allocator<PointType> > DynamicPoints = DynamicPointsPtr->points;
    std::vector<std::vector<PointType>> clusters = canopy_clustering(DynamicPoints,t1,t2);
    ROS_INFO_STREAM("\033[1;32m -- clusters number "<< clusters.size()<<   "\033[0m");
    pcl::PointCloud<PointType>::Ptr NewCloud(new pcl::PointCloud<PointType>);
//    VectorPoints2PointPtr(clusters,NewCloud);
    PointsVisual(clusters,NewCloud);
}