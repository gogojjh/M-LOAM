#include <iostream>
#include <vector>
#include <boost/smart_ptr.hpp>

using namespace std;

int main(int argc, char **argv) 
{
    int *p = new int;
    *p = 1;
    int *q = p;
    // cout << *p << " " << *q << endl;
    delete q;
    // cout << *p << endl;
    // delete p;
}