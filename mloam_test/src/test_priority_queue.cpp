#include <queue>
#include <iostream>
#include <random>

class FeatureWithScore
{
public:
    FeatureWithScore(const int &idx, const double &score)
        : idx_(idx), score_(score) {}

    bool operator < (const FeatureWithScore &fws) const
    {
        return this->score_ < fws.score_;
    }

    size_t idx_;
    double score_;
};

struct CustomCompare
{
    bool operator()(const FeatureWithScore &lhs, const FeatureWithScore &rhs)
    {
        return lhs.score_ > rhs.score_;
    }
};

struct rotCmp
{
    bool operator()(const size_t &r, const size_t &l)
    {
        return (l > r);
    }
};

int main() 
{
    // std::priority_queue<FeatureWithScore, std::vector<FeatureWithScore>, std::less<FeatureWithScore> > q;
    // for (size_t i = 0; i < 10; i++)
    // {
    //     double score = rand() % 100;
    //     FeatureWithScore f(i, score);
    //     q.push(f);
    // }
    // while (!q.empty())
    // {
    //     std::cout << q.top().score_ << std::endl;
    //     q.pop();
    // }

    // size_t n = 0;
    // std::vector<int> feature_visited(10, -1);
    // for (size_t i = 0; i < 10; i++)
    // {
    //     // std::cout << feature_visited[i] << std::endl;
    //     std::cout << (feature_visited[i] < int(n)) << std::endl;
    // }

    std::priority_queue<size_t, std::vector<size_t>, rotCmp> q;
    for (size_t i = 0; i < 10; i++)
    {
        size_t d = size_t(rand() % 30);
        q.push(d);
    }
    while (!q.empty())
    {
        std::cout << q.top() << std::endl;
        q.pop();
    }
    return 0;
}