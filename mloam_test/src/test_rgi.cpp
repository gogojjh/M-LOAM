#include <iostream>

#include "common/random_generator.hpp"

using namespace std;

int main()
{
    common::RandomGeneratorInt<size_t> rgi;
    for (size_t i = 0; i < 30; i++)
        std::cout << rgi.geneRandUniform(0, 100) << std::endl;

    common::RandomGeneratorFloat<double> rgf;
    for (size_t i = 0; i < 30; i++)
        std::cout << rgf.geneRandNormal(0, 1.0) << std::endl;        
    return 0;
}