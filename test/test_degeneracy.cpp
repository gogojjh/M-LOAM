
// zhang
if (iterCount == 0)
{
  cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

  cv::eigen(matAtA, matE, matV);
  matV.copyTo(matV2);

  isDegenerate = false;
  float eignThre[6] = {100, 100, 100, 100, 100, 100};
  for (int i = 5; i >= 0; i--) {
    if (matE.at<float>(0, i) < eignThre[i]) {
      for (int j = 0; j < 6; j++) {
        matV2.at<float>(i, j) = 0;
      }
      isDegenerate = true;
    } else {
      break;
    }
  }
  matP = matV.inv() * matV2;
}

if (isDegenerate)
{
  cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
  matX.copyTo(matX2);
  matX = matP * matX2;
}

// jjiao
Eigen::Matrix<double, 6, 6> mat_H = mat_JtJ.block(6*i, 6*i, 6, 6);
Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> > esolver(mat_H);
Eigen::Matrix<double, 1, 6> mat_E = esolver.eigenvalues().real() / 400.0; // 6*1
Eigen::Matrix<double, 6, 6> mat_V_f = esolver.eigenvectors().real(); // 6*6, column is the corresponding eigenvector
Eigen::Matrix<double, 6, 6> mat_V_p = mat_V_f;
// std::cout << "H:" << std::endl << mat_H;
for (auto j = 0; j < mat_E.cols(); j++)
{
    if (mat_E(0, j) < MAP_EIG_THRE)
    {
        mat_V_p.col(j) = Eigen::Matrix<double, 6, 1>::Zero();
        // local_param_ids[i]->is_degenerate_ = true;
    } else
    {
        break;
    }
}
std::cout << i << ": D factor: " << mat_E(0, 0) << ", D vector: " << mat_V_f.col(0).transpose() << std::endl;
Eigen::Matrix<double, 6, 6> mat_P = (mat_V_f.transpose()).inverse() * mat_V_p.transpose(); // 6*6
assert(mat_P.rows() == 6);
