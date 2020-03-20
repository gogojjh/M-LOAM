#include <iostream>
#include <vector>
#include <unistd.h>
#include <stdio.h>

using namespace std;

int main()
{
    vector<int> a{1,2,3,4,5,6,7};
    // for (auto iter = a.begin(); iter != a.begin() + 3; iter++) cout << *iter << endl;

    while (true)
    {
        cout << "1 ";
        usleep(10000);
        if (getchar()) cout << "enter:" << endl;
    }

    return 0;
}