//
// Created by zhuhongwei on 2023/4/1.
//
#pragma once

#include <iostream>
#include <algorithm>
#include<pcl/point_cloud.h>
#include<pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <removert/utility.h>

class KMeans
{
private:
    unsigned int max_iteration_;
    const unsigned int cluster_num_;//k
    double pointsDist(const PointType& p1, const PointType& p2);

public:

    pcl::PointCloud<PointType>::Ptr centre_points_;
    //KMeans() = default;
    KMeans(unsigned int k, unsigned int max_iteration);
    void kMeans(const pcl::PointCloud<PointType>::Ptr& cloud, std::vector<pcl::PointCloud<PointType>> &cluster_cloud1);

    ~KMeans(){}
};

