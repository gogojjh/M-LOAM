#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[])
{
    // {
    // TicToc t_eval_H;
    // MatrixXd J = MatrixXd::Random(2000, 10);
    // MatrixXd Jt = J.transpose();
    // MatrixXd H = Jt * J;
    // printf("dense: t_eval_H: %fms\n", t_eval_H.toc());
    // }

    // {
    // TicToc t_eval_H;
    // SparseMatrix<double, RowMajor> J;
    // J.resize(2000, 10);
    // J.coeffRef(1, 1) = 10.5;
    // SparseMatrix<double, RowMajor> Jt = J.transpose();
    // MatrixXd H = Jt * J;
    // cout << H << endl;
    // printf("sparse: t_eval_H: %fms\n", t_eval_H.toc());
    // }

    double *x = new double [7];
    x[0] = 1;
    x[1] = 2;
    x[2] = 3;
    x[3] = 0;
    x[4] = 0;
    x[5] = 0;
    x[6] = 1;

    Eigen::Map<Eigen::Vector3d> p(x);
    Eigen::Map<Eigen::Quaterniond> q(x + 3);
    std::cout << p.transpose() << std::endl; 
    std::cout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;

    return 0;
}