#include <stack>
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
    stack<int> a;
    for (int i = 0; i < 10; i++) a.push(i);
    while (!a.empty())
    {
        cout << a.top() << endl;
        a.pop();
    }

    return 0;
}