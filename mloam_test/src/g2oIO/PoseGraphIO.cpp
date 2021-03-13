//
// Created by echo on 2019/11/11.
//

#include "PoseGraphIO.h"

void PoseGraphIO::getPose()
{
}

void PoseGraphIO::saveGraph(std::string g2o_path)
{
    g2o::SparseOptimizer optimizer; // 图模型
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    optimizer.setAlgorithm(solver); // 设置求解器
    optimizer.setVerbose(true);     // 打开调试输出
    //把全部的pose存起来
    Eigen::Matrix<double, 6, 6> _information = Eigen::Matrix<double, 6, 6>::Identity(); //信息矩阵

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> _odom_buffer_t;
    _odom_buffer_t = _odom_buffer;
    for (int i = 0; i < _odom_buffer_t.size(); ++i)
    {

        g2o::VertexSE3 *v = new g2o::VertexSE3();

        if (i == 0)
        {
            v->setFixed(true);
        }
        //顶点
        v->setId(i);
        v->setEstimate(_odom_buffer_t[i]);
        optimizer.addVertex(v);

        // 生成边
        if (i > 0)
        {
            Eigen::Matrix4d t_e_m;
            t_e_m = _odom_buffer_t[i - 1].matrix();
            Eigen::Isometry3d t_e;
            t_e = t_e_m.inverse() * _odom_buffer_t[i].matrix();
            g2o::EdgeSE3 *e = new g2o::EdgeSE3();
            e->setVertex(0, optimizer.vertices()[i - 1]); //debug
            e->setVertex(1, optimizer.vertices()[i]);     //debug
            e->setMeasurement(t_e);                       //debug
            e->setInformation(_information);
            optimizer.addEdge(e);
        }
    }
    /*	optimizer.initializeOptimization();
	optimizer.optimize(30);*/
    optimizer.save(g2o_path.c_str());
    std::cout << "result saved!" << std::endl;
}

void PoseGraphIO::insertPose(Eigen::Isometry3d pose)
{

    _odom_buffer.push_back(pose);
}

std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
PoseGraphIO::getEigenPoseFromg2oFile(std::string &g2ofilename)
{
    // std::cout << "Reading the g2o file ~" << std::endl;
    std::ifstream fin(g2ofilename);
    // std::cout << "G2o opened" << std::endl;
    if (!fin)
    {
        std::cerr << "file " << g2ofilename << " does not exist." << std::endl;
        std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> vT;
        vT.resize(1);
        vT[0] = Eigen::Matrix4d::Identity(4, 4);
        return vT;
    }

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> vT;
    while (!fin.eof())
    {
        std::string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT")
        {
            g2o::VertexSE3 v;

            int index = 0;
            fin >> index;
            v.setId(index);
            v.read(fin);
            Eigen::Isometry3d T = v.estimate();
            vT.push_back(T);
        }
        else if (name == "EDGE_SE3:QUAT")
        {
            continue;
        }
        else
            continue;
        if (!fin.good())
            break;
    }
    // std::cout << "read total " << vT.size() << " vertices\n";
    return vT;
}