#include <iostream>
#include <unistd.h>

#include "common/timing.hpp"

using namespace std;
using namespace common;

int main(int argc, char* argv[])
{
    for (size_t i = 0; i < 3; i++)
    {
        timing::Timer test_timer("test_time");
        sleep(i);
        test_timer.Stop();
    }
    std::cout << timing::Timing::GetNumSamples("test_time") << std::endl;
    std::cout << timing::Timing::GetTotalSeconds("test_time") << std::endl;
    std::cout << timing::Timing::GetMeanSeconds("test_time") << std::endl;
    std::cout << timing::Timing::GetHz("test_time") << std::endl;
    std::cout << timing::Timing::GetNewestTime("test_time") << std::endl;
    return 0;
}
