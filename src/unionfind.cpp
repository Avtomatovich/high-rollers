#include "unionfind.h"

#include <numeric>


UnionFind::UnionFind(size_t n)
{
    reset(n);
}

void UnionFind::reset(size_t n)
{
    // set each parent of each set to self
    _parent.clear();
    _parent.resize(n);
    std::iota(_parent.begin(), _parent.end(), 0);

    // set size of each set to 1
    _size.assign(n, 1);
}

size_t UnionFind::find(size_t i)
{
    size_t pi = i, j = i;

    // fetch parent of set i
    while (pi != _parent[pi]) pi = _parent[pi];

    // // path compression
    // if parent of set i is not reached
    while (_parent[j] != _parent[i]) {
        // cache parent of set j
        size_t pj = _parent[j];
        // set parent of set j to grandparent of set j
        _parent[j] = _parent[pj];
        // continue to parent of set j
        j = pj;
    }

    return pi;
}

void UnionFind::join(size_t i, size_t j)
{
    // fetch parents of set i and set j
    size_t pi = find(i), pj = find(j);

    // return if already joined
    if (pi == pj) return;

    // if num of vals in set i is greater
    if (_size[pi] > _size[pj]) {
        // set parent of set j to parent of set i
        _parent[pj] = pi;
        // increase size of set i by size of set j
        _size[pi] += _size[pj];

    // if num of vals in set j is greater
    } else {
        // set parent of set i to parent of set j
        _parent[pi] = pj;
        // increase size of set j by size of set i
        _size[pj] += _size[pi];
    }
}

size_t UnionFind::size(size_t i)
{
    return (0 <= i && i < _size.size()) ? _size[i] : 0;
}
