#include <iostream>

using namespace std;

int main() 
{
    double para_pose[6] = {1, 2, 3, 4, 5, 6};
    const double **param = new const double *[1];
    param[0] = para_pose;
    for (size_t i = 0; i < 6; i++) cout << param[0][i] << endl;
    return 0;
}