#include <vector>
#include <random>
#include <set>
#include <iostream>

using std::set;
using std::mt19937_64;
using std::random_device;
using std::uniform_int_distribution;
using std::vector;
using std::cout;
using std::cerr;
using std::endl;

int main()
{
    // Amount to create;
    unsigned long long amount = 10;

    // Generate a number from 1 to 1,000,000
    unsigned long long min = 1;
    unsigned long long max = 12;

    // If the amount is greater than the max then this will fail
    if (amount > max)
    {
        cerr << "Amount must be less than max." << endl;
        return -1;
    }

    // Initialize the random_device
    random_device rd;

    // Seed the engine
    mt19937_64 generator(rd());

    // Specify the range of numbers to generate, in this case [min, max]
    uniform_int_distribution<unsigned long long> dist{min, max};

    // Create a set to hold the random numbers
    set<unsigned long long> results;

    // Generate the random numbers
    while (results.size() != amount)
    {
        results.insert(dist(generator));
    }

    // Display the results
    for(auto &n : results)
    {
        cout << n << " ";
    }
    cout << endl;
    return 0;
}