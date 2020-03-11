#include <iostream>
#include <algorithm>
#include <vector>
#include <queue>
#include <map>

using namespace std;

// bool cmp(int x, int y)
// {
//     return (x > y);
// }

// bool cmp(vector<int> x, vector<int> y)
// {
//     return (x[0] > y[0]);
// }

struct cmp
{
    bool operator()(const pair<vector<int>, int> &x, const pair<vector<int>, int> &y) { return x.first[0] > y.first[0]; }
};

int main()
{

    // Initializing a vector
    vector<int> v1;
    v1.push_back(20);
    v1.push_back(30);
    v1.push_back(40);
    v1.push_back(25);
    v1.push_back(15);

    vector<int> v2;
    v2.push_back(10);
    v2.push_back(30);
    v2.push_back(40);
    v2.push_back(25);
    v2.push_back(15);

    std::priority_queue< pair<vector<int>, int>, vector< pair<vector<int>, int> >, cmp> v_list;
    v_list.push(make_pair(v1, 1));
    v_list.push(make_pair(v2, 2));

    // std::cout << v_list.size() << std::endl;

    while (v_list.empty() == false)
    {
        pair<vector<int>, int> v = v_list.top();
        cout << v.second << endl;
        cout << v.first[0] << endl;
        v_list.pop();
    }

    // cout << "The minimum element of heap is : ";
    // cout << v1.front() << endl;

    // v1.push_back(5);
    // push_heap(v1.begin(), v1.end(), cmp);
    // cout << "The minimum element of heap after push is : ";
    // cout << v1.front() << endl;

    // pop_heap(v1.begin(), v1.end(), cmp);
    // std::cout << v1.size() << std::endl;
    // v1.pop_back();
    // std::cout << v1.size() << std::endl;
    // cout << "The minimum element of heap after pop is : ";
    // cout << v1.front() << endl;

    // for (size_t i = 0; i < v1.size(); i++) std::cout << v1[i] << std::endl;

    return 0;
}