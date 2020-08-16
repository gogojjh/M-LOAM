// A tools that can generate ture random variable, with modern c++ approach
// Author: Jiarong Lin
// In the develop of this tools, I refer to :
// [1]. https://www.fluentcpp.com/2019/05/24/how-to-fill-a-cpp-collection-with-random-values/
// [2]. https://www.zhihu.com/question/3999110/answer/84257121

#ifndef __TOOLS_RANDOM_HPP__
#define __TOOLS_RANDOM_HPP__
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <random>

namespace common
{
    template < typename T >
    struct RandomGeneratorFloat
    {
        std::random_device                 random_device;
        std::mt19937                       m_random_engine;
        std::uniform_real_distribution< T > m_dist;
        std::normal_distribution< T > m_dist_normal;
        RandomGeneratorFloat(): m_random_engine( std::random_device{}() )
        {};
        ~RandomGeneratorFloat(){};

        T geneRandUniform( T low = 0.0, T hight = 100.0 )
        {
            m_dist = std::uniform_real_distribution< T >( low, hight );
            return m_dist( m_random_engine );
        }

        T geneRandNormal( T mean = 0.0, T std = 1.0 )
        {
            m_dist_normal = std::normal_distribution< T >( mean, std );
            return m_dist_normal( m_random_engine );
        }

        T *geneRandUniformArray( T low = 0.0, T hight = 100.0, size_t numbers= 100 )
        {
            T *res = new T[ numbers ];
            m_dist = std::uniform_real_distribution< T >( low, hight );
            for ( size_t i = 0; i < numbers; i++ )
            {
                res[ i ] = m_dist( m_random_engine );
            }
            return res;
        }
    };

    template < typename T >
    struct RandomGeneratorInt
    {
        std::random_device                 random_device;
        std::mt19937                       m_random_engine;
        std::uniform_int_distribution< T > m_dist;
        RandomGeneratorInt(): m_random_engine( std::random_device{}() )
        {};
        ~RandomGeneratorInt(){};

        T geneRandUniform( T low = 0, T hight = 100 )
        {
            m_dist = std::uniform_int_distribution<T>(low, hight);
            return m_dist( m_random_engine );
        }

        T *geneRandUniformArray( T low = 0, T hight = 100, size_t numbers = 100 )
        {
            T *res = new T[ numbers ];
            m_dist = std::uniform_int_distribution< T >( low, hight );
            for ( size_t i = 0; i < numbers; i++ )
            {
                res[ i ] = m_dist( m_random_engine );
            }
            return res;
        }
        
        T* geneRandArrayNoRepeat( T low,  T high, T k)
        {
            T                  n = high - low;
            T *                res_array = new T[ k ];
            std::vector<T> foo;
            foo.resize( n );
            for ( T i = 1; i <= n; ++i ) foo[ i ] = i + low;
            std::shuffle( foo.begin(), foo.end(), m_random_engine );
            for ( T i = 0; i < k; ++i )
            {
                res_array[ i ] = foo[ i ];
                // std::cout << foo[ i ] << " ";
            }
            return res_array;
        }
    };

}

#endif