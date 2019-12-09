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

// TODO: initialization
    // RANSAC-based approach
//    const Eigen::Quaterniond &q_zyx = calib_ext_[idx_data].q_;
//    int ransac_split = 3;
//    int ransac_loop = 10;
//    int ransac_size = pose_cnt_ / ransac_split;
//    double e_th = 1e6;
//    Eigen::Vector3d x_opt;
//    srand(time(0));
//    for (auto k = 0; k < ransac_loop; k++)
//    {
//        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(ransac_size * 3, 3);
//        Eigen::MatrixXd b = Eigen::MatrixXd::Zero(ransac_size * 3, 1);
//        for (auto i = 0; i < ransac_size; i++)
//        {
//            auto j = rand() % pose_cnt_;
//            Pose &pose_ref = v_pose_[idx_ref][indices_[idx_data][j]];
//            Pose &pose_data = v_pose_[idx_data][indices_[idx_data][j]];
//            AngleAxisd ang_axis_ref(pose_ref.q_);
//            AngleAxisd ang_axis_data(pose_data.q_);
//            double t_dis = abs(pose_ref.t_.dot(ang_axis_ref.axis()) - pose_data.t_.dot(ang_axis_data.axis()));
//            double huber = t_dis > 0.05 ? 0.05 / t_dis : 1.0;
//            A.block<3, 3>(i * 3, 0) = huber * (pose_ref.q_.toRotationMatrix() - Eigen::Matrix3d::Identity());
//            b.block<3, 1>(i * 3, 0) = q_zyx * pose_data.t_ - pose_ref.t_;
//        }
//        Eigen::Vector3d x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
//        double e = Eigen::MatrixXd(A*x - b).norm();
//        if (e < e_th)
//        {
//            e_th = e;
//            x_opt = x;
//            std::cout << "error: " << e << "| " << x_opt.transpose() << std::endl;
//        }
//    }
//    calib_ext_[idx_data] = Pose(q_zyx, x_opt);
//    return true;

//    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(pose_cnt_ * 3, 3);
//    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(pose_cnt_ * 3, 1);
//    for (size_t i = 0; i < pose_cnt_; i++)
//    {
//        Pose &pose_ref = v_pose_[idx_ref][indices_[idx_data][i]];
//        Pose &pose_data = v_pose_[idx_data][indices_[idx_data][i]];
//        AngleAxisd ang_axis_ref(pose_ref.q_);
//        AngleAxisd ang_axis_data(pose_data.q_);
//        double t_dis = abs(pose_ref.t_.dot(ang_axis_ref.axis()) - pose_data.t_.dot(ang_axis_data.axis()));
//        double huber = t_dis > 0.05 ? 0.05 / t_dis : 1.0;
//        A.block<3, 3>(i * 3, 0) = huber * (pose_ref.q_.toRotationMatrix() - Eigen::Matrix3d::Identity());
//        b.block<3, 1>(i * 3, 0) = q_zyx * pose_data.t_ - pose_ref.t_;
//    }
//
//    Eigen::Vector3d x, x_prev, x_delta;
//    x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
//    calib_ext_[idx_data] = Pose(q_zyx, x);
//    return true;

    // TODO: using singular value to check constraints
//    Eigen::Vector3d x, x_prev, x_delta;
//    x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
////    x_prev = calib_ext_[idx_data].t_;
////    x_delta = x - x_prev;
//    calib_ext_[idx_data] = Pose(q_zyx, x);
//    Eigen::MatrixXd A_aug = Eigen::MatrixXd::Zero(A.rows(), A.cols() + 1);
//    A_aug.leftCols(A.cols()) = A;
//    A_aug.rightCols(b.cols()) = b;
//    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A_aug, Eigen::ComputeFullU | Eigen::ComputeFullV);
//    Eigen::Vector3d pos_cov = svd.singularValues().head<3>();
//    v_pos_cov_[idx_data].push_back(pos_cov(1));
//    printf("pose_cnt:%d, pos_cov:%f **********\n", pose_cnt_, pos_cov(1));
//    if (pose_cnt_ >= WINDOW_SIZE && pos_cov(1) > 0.7) // converage
//        return true;
//    else
//        return false;

//    if (x_prev.norm() != 0)
//    {
//        double d = 1.0 * x_delta.norm() / x_prev.norm();
//        printf("||dx %||: %f\n", d);
//        if (pose_cnt_ >= WINDOW_SIZE && d < 1e-2)
//            return true;
//    }
//    return false;