#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <deque>
#include <numeric>

template<typename T, unsigned int MAX>
class smoother
{
public:
    void add(T v)
    {
        data.push_back(v);
        if (data.size() > MAX)
            data.pop_front();
    }

    T get()
    {
        return std::accumulate(data.begin(), data.end(), 0.0) / data.size();
    }

private:
    std::deque<T> data;
};

#endif // SMOOTHER_H
