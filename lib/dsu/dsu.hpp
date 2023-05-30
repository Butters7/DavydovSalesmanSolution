#pragma once

#include <iostream>
#include <vector>
#include <map>

class DSU {
public:
	DSU(size_t size);

	size_t find_set(size_t u);

	void union_set(size_t u, size_t v);

private:
	friend std::ostream& operator<<(std::ostream& out, DSU& dsu);

	std::vector<size_t> parent_;
	std::vector<size_t> rank_;
};
