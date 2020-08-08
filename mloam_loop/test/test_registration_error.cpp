// rosrun mloam_loop test_evaluation baseline_data/

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

Eigen::Matrix4f ReadTrans(const char *filename)
{
    Eigen::Matrix4f temp;
    temp.fill(0);
    FILE *fid = fopen(filename, "r");
    for (int j = 0; j < 4; j++)
    {
        float a, b, c, d;
        fscanf(fid, "%f %f %f %f", &a, &b, &c, &d);
        temp(j, 0) = a;
        temp(j, 1) = b;
        temp(j, 2) = c;
        temp(j, 3) = d;
    }
    return temp;
}

int main(int argc, char *argv[])
{
    Eigen::Matrix4f T_gt = ReadTrans(std::string(std::string(argv[1]) + "gt.log").c_str());
    Eigen::Matrix4f T_icp = ReadTrans(std::string(std::string(argv[1]) + "output_icp.txt").c_str());
    Eigen::Matrix4f T_fgr = ReadTrans(std::string(std::string(argv[1]) + "output_fgr.txt").c_str());
    std::vector<float> r_error;
    std::vector<float> t_error;

    Eigen::Quaternionf q_gt(T_gt.block<3, 3>(0, 0));
    Eigen::Quaternionf q_icp(T_icp.block<3, 3>(0, 0));
    Eigen::Quaternionf q_fgr(T_fgr.block<3, 3>(0, 0));

    r_error.push_back(q_gt.angularDistance(q_icp));
    t_error.push_back((T_gt.block<3, 1>(0, 3) - T_icp.block<3, 1>(0, 3)).norm());

    r_error.push_back(q_gt.angularDistance(q_fgr));
    t_error.push_back((T_gt.block<3, 1>(0, 3) - T_fgr.block<3, 1>(0, 3)).norm());

    FILE *fid = fopen(std::string(std::string(argv[1]) + "transform_error.txt").c_str(), "w");
    fprintf(fid, "%f %f\n", r_error[0], r_error[1]);
    fprintf(fid, "%f %f\n", t_error[0], t_error[1]);
    fclose(fid);

    printf("r_error: icp: %f, fgr: %f\n", r_error[0], r_error[1]);
    printf("t_error: icp: %f, fgr: %f\n", t_error[0], t_error[1]);
}

