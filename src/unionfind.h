#ifndef UNIONFIND_H
#define UNIONFIND_H


#include <vector>

class UnionFind
{
public:
    UnionFind(size_t n);

    void reset(size_t n);
    size_t find(size_t i);
    void join(size_t i, size_t j);

    int size(size_t i);

private:
    std::vector<size_t> _parent;
    std::vector<int> _size;
};

#endif // UNIONFIND_H
