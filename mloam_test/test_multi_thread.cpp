#include <thread>
#include <iostream>
#include <vector>
#include <mutex>
#include <unistd.h>
#include <queue>
#include <omp.h>

using namespace std;

queue<vector<long int> > data_buf;
mutex m_buf;

void sync_process()
{
    m_buf.lock();
    vector<long int> v;
    for (size_t i = 0; i < 10000000; i++) v.push_back(i);
    // for (size_t i = 0; i < 100; i++)
    // {
    //     data_buf.push(v);
    // }
    m_buf.unlock();
}

int main(int argc, char *argv[])
{
    // thread sync_thread(sync_process);
    // while (1)
    // {
    //     if (!data_buf.empty())
    //     {
    //         cout << *(data_buf.front().end()-1) << std::endl;
    //         m_buf.lock();
    //         data_buf.pop();
    //         m_buf.unlock();
    //         std::this_thread::sleep_for(std::chrono::milliseconds(2));
    //     }
    // }
    // sync_thread.join();

    // cout << "thread num: " << omp_get_thread_num() << endl;
    // cout << "procs num: " << omp_get_num_procs() << endl;

    int *v = new int[100];
    #pragma omp parallel for num_threads(4)
    for (int i = 0; i < 100; i++)
    {
        v[i] = i;
        // cout << &i << endl;
    }
    for (size_t i = 0; i < 100; i++) 
        cout << *(v + i) << endl;
    return 0;    
}