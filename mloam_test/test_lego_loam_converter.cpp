#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        cerr << "Usage: ./test_lego_loam_converter trajectory_results/real_vehicle/rv01/";
        return 0;
    }
    string data_path = argv[1];
    // cout << data_path << endl;
    fstream odom_file_in(std::string(data_path + std::string("stamped_legoloam_map_estimate.txt")).c_str(), ios::in);
    fstream odom_file_out(std::string(data_path + std::string("stamped_legoloam_map_estimate_convert.txt")).c_str(), ios::out);
    if (odom_file_in.is_open() && odom_file_out.is_open())
    {
        double timestamp, x, y, z, qx, qy, qz, qw;
        while (!odom_file_in.eof())
        {
            odom_file_in >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw;
            odom_file_out.precision(15);
            odom_file_out << timestamp << " "; 
            odom_file_out.precision(8);
            odom_file_out << z << " " << x << " " << y << " " << qz << " " << qx << " " << qy << " " << qw << endl;
        }
    }
    odom_file_in.close();
    odom_file_out.close();
    return 0;
}