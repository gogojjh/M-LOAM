/*************************************************************************
	> File Name: test/test_thread.cpp
	> Author: jjiao
	> Mail: jiaojh1994@gmail.com
	> Created Time: Thu 26 Sep 2019 07:49:19 PM HKT
 ************************************************************************/

#include <iostream>
#include <thread>

using namespace std;

class X 
{
public: 
    void do_lengthy_work(int x)
    {
        std::cout << x << std::endl;
    }    
};

int main()
{
    X my_x;
    int num(0);
    std::thread t(&X::do_lengthy_work, &my_x. num);
}
