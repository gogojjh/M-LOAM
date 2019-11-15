/*************************************************************************
	> File Name: test/test_sqr.cpp
	> Author: jjiao
	> Mail: jiaojh1994@gmail.com
	> Created Time: Fri 09 Aug 2019 02:58:07 PM HKT
 ************************************************************************/

#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
using namespace std;

struct Pose
{
    int a;
    void add(int x)
    {
        a += x;
    }
};

bool test_print()
{
    cout << "test_if" << endl;
    return true;
}

int main()
{
    double a[10];
    a[0] = 1;
    a[1] = 2;
    a[2] = 3;
    std::cout << a << std::endl;

    getchar();

    //std::cout << sqrt(0) << std::endl;
    float aa = 0.123456798;
    printf("%.3f", aa);

    //std::cin.get();

    std::vector<int> v;
    v.clear();
    v.push_back(1);
    v.push_back(2);
    v.push_back(3);
    v.resize(2);
    cout << v.size() << endl;

    //std::cin.get();

    Pose pose;
    pose.a = 1;
    pose.add(4);
    cout << pose.a << endl;

    bool b = false;
    if ((b) && (test_print()))
    {
        cout << "finish test_if" << endl;
    }

    std::vector<int> vec(5, 1);
    //vec.push_back(1);
    //vec.push_back(2);
    //vec.push_back(3);
    cout << vec.size() << endl;
    cout << vec[0] << vec[1] << endl;

    vec.resize(0);
    cout << vec.size() << endl;

    std::vector<double> corner(5);
    std::cout << corner.size() << ", " << corner[0] << std::endl;

    return 0;
}

