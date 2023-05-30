#include "dsu.hpp"

DSU::DSU(size_t size) : parent_(size), rank_(size, 1) {
    for (size_t i = 0; i < size; ++i) {
        parent_[i] = i;
    }
}

size_t DSU::find_set(size_t u) {
    if (u == parent_[u])
        return u;
    return parent_[u] = find_set(parent_[u]);
}

void DSU::union_set(size_t u, size_t v) {
    u = find_set(u);
    v = find_set(v);

    if (u != v) {
        if (rank_[u] < rank_[v])
            std::swap(u, v);

        parent_[v] = u;
        rank_[u] += rank_[v];
    }
}

std::ostream& operator<<(std::ostream& out, DSU& dsu) {
    std::map<size_t, std::vector<size_t>> sets;

    for (auto i = 0; i < dsu.parent_.size(); ++i) {
        sets[dsu.find_set(i)].push_back(i);
    }

    for (auto &kv : sets) {	
        out << kv.first << "[rank = " << dsu.rank_[kv.first] << "]: ";
        for (auto i = 0; i < kv.second.size(); ++i) {
            out << kv.second[i];
            if (i != kv.second.size() - 1)
                out << ", ";
        }
        out << std::endl;
    }

    return out;
}
