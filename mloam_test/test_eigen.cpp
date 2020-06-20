#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include "../src/utility/tic_toc.h"

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[])
{
    {
    TicToc t_eval_H;
    MatrixXd J = MatrixXd::Random(2000, 10);
    MatrixXd Jt = J.transpose();
    MatrixXd H = Jt * J;
    printf("dense: t_eval_H: %fms\n", t_eval_H.toc());
    }

    {
    TicToc t_eval_H;
    SparseMatrix<double, RowMajor> J;
    J.resize(2000, 10);
    J.coeffRef(1, 1) = 10.5;
    SparseMatrix<double, RowMajor> Jt = J.transpose();
    MatrixXd H = Jt * J;
    cout << H << endl;
    printf("sparse: t_eval_H: %fms\n", t_eval_H.toc());
    }

    return 0;
}