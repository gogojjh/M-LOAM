//
// Created by echo on 2019/11/11.
//
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>

#ifndef POSEGRAPHTOOLS_POSEGRAPHIO_H
#define POSEGRAPHTOOLS_POSEGRAPHIO_H

class PoseGraphIO
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    PoseGraphIO(){

    };
    void getPose();
    void insertPose(Eigen::Isometry3d pose);
    void saveGraph(std::string g2o_path);
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> getEigenPoseFromg2oFile(std::string &g2ofilename);
    void saveCloud();

private:
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> _odom_buffer;

    int index = 0;
    Eigen::Isometry3d prev;
    std::vector<g2o::VertexSE3> vertexs;
    std::vector<g2o::EdgeSE3> edges;
    /*	g2o::VertexSE3 *v;
	g2o::EdgeSE3 *e ;*/
    /*	bool begin_pose = true;*/
protected:
};

#endif //POSEGRAPHTOOLS_POSEGRAPHIO_H