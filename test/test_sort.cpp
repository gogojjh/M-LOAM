#include <vector>
#include <iostream>
#include <numeric>
#include <algorithm>


bool myfunction(int i, int j) { return (i < j); } //升序排列

int main()
{
    std::vector<int> data(8);
    data[0] = 2;
    data[1] = 3;
    data[2] = 1;
    data[3] = 5;
    data[4] = 9;
    data[5] = 2;
    data[6] = 4;
    data[7] = 6;
    std::vector<size_t> ind(8);
    ind[0] = 0;
    ind[1] = 1;
    ind[2] = 2;
    ind[3] = 3;
    ind[4] = 4;
    ind[5] = 5;
    ind[6] = 6;
    ind[7] = 7;

    std::sort(ind.begin(), ind.end(),
              [&data](size_t a, size_t b) { return data[a] < data[b]; });

    for (size_t i = 0; i < data.size(); i++) std::cout << data[i] << std::endl;

    for (size_t i = 0; i < ind.size(); i++) std::cout << ind[i] << std::endl;
    return 0;
}

