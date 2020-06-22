#include <iostream>
#include <iomanip>

using namespace std;

int main(int argc, char* argv)
{
    // double a = 1547131046560135.0;
    // printf("%f\n", a);

    // float b = a;
    // std::cout << std::fixed << std::setprecision(3) << b << std::endl;

    int a = 1;
    bool ground_flag = (a == 1) ? true : false;
    std::cout << ground_flag << std::endl;

    return 0;
}