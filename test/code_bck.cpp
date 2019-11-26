// Estimator::evalDegenracy()
// // -----------------
// {
//     Eigen::MatrixXd mat_J = mat_J_raw.block(0, 6*(OPT_WINDOW_SIZE + 1), mat_J_raw.rows(), 6*NUM_OF_LASER);
//     Eigen::MatrixXd mat_Jt = mat_J.transpose(); // A^T
//     Eigen::MatrixXd mat_JtJ = mat_Jt * mat_J; // A^TA
//     Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> esolver(mat_JtJ);
//     Eigen::MatrixXd mat_E = esolver.eigenvalues().real(); // 12*1
//     Eigen::MatrixXd mat_V = esolver.eigenvectors().real(); // 12 * 12
//     std::cout << "##### Calib degeneracy factor: " << mat_E(6, 0) << ", vector: " << mat_V.row(6) << std::endl;
// }
//
// // -----------------
// {
//     Eigen::MatrixXd mat_J = mat_J_raw.block(0, 0, mat_J_raw.rows(), 6*(OPT_WINDOW_SIZE + 1));
//     Eigen::MatrixXd mat_Jt = mat_J.transpose(); // A^T
//     Eigen::MatrixXd mat_JtJ = mat_Jt * mat_J; // A^TA
//     Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> esolver(mat_JtJ);
//     Eigen::MatrixXd mat_E = esolver.eigenvalues().real(); // 36*1
//     Eigen::MatrixXd mat_V = esolver.eigenvectors().real();
//     std::cout << "##### Odom degeneracy factor: " << mat_E(6, 0) << std::endl;
// }

// -----------------
// {
    // Eigen::MatrixXd mat_A = Eigen::MatrixXd::Zero(mat_A_raw.rows(), 6*OPT_WINDOW_SIZE + 6*(NUM_OF_LASER - 1));
    // mat_A.block(0, 0, mat_A_raw.rows(), 6*OPT_WINDOW_SIZE) = mat_A_raw.block(0, 6, mat_A_raw.rows(), 6*OPT_WINDOW_SIZE);
    // mat_A.block(0, 6*OPT_WINDOW_SIZE, mat_A_raw.rows(), 6*(NUM_OF_LASER - 1)) = mat_A_raw.block(0, 6*(OPT_WINDOW_SIZE + 1) + 6, mat_A_raw.rows(), 6*(NUM_OF_LASER - 1)); // M*36
    // Eigen::MatrixXd &mat_A = mat_A_raw;


    // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> esolver(mat_AtA);
    // Eigen::MatrixXd mat_E = esolver.eigenvalues().real(); // 48*1
    // Eigen::MatrixXd mat_V = esolver.eigenvectors().real();
    // Eigen::MatrixXd mat_V_update = mat_V;
    // is_degenerate_ = false;
    // // float eign_thre[6] = {10, 10, 10, 10, 10, 10};
    // double eigen_thre = 100;
    // for (int i = 0; i < mat_E.rows(); i++)
    // {
    //     if (mat_E(i, 0) < eigen_thre)
    //     {
    //         mat_V_update.row(i) = Eigen::MatrixXd::Zero(1, mat_V_update.cols());
    //         is_degenerate_ = true;
    //     } else
    //     {
    //         break;
    //     }
    // }
    // Eigen::MatrixXd mat_P = mat_V_update * mat_V.inverse();
    // printf("##### mat_P size: %d, %d\n", mat_P.rows(), mat_P.cols()); // 48*48
    // assert(local_param_ids.size() * 6 == mat_P.rows());
    // if (is_degenerate_)
    // {
    //     for (int i = 0; i < local_param_ids.size(); i++)
    //     {
    //         local_param_ids[i]->V_update_ = mat_P.block(0, i*6, mat_P.rows());
    //     }
    // }
// }
// if (is_degenerate_)
// {
//   Eigen::Matrix<float, 6, 1> mat_X2;
//   mat_X2 = mat_X;
//   mat_X = mat_P * mat_X2;
// }

// ----------------- test eigenvalue and eigenvector
// std::cout << "mat_H: " << std::endl << mat_H << std::endl;
// std::cout << "mat_E: " << mat_E << std::endl;
// std::cout << "mat_V: " << std::endl << mat_V << std::endl;
// std::cout << "left: " << std::endl << (mat_H * mat_V.col(1)).transpose() << std::endl;
// std::cout << "right: " << std::endl << (mat_E(0, 1) * mat_V.col(1)).transpose() << std::endl;
// std::cout << i << ": D factor: " << mat_E(0, 0) << ", D vector: " << mat_V_f.col(0).transpose() << std::endl;

// openmp example
// #pragma omp parallel for num_threads(4)
