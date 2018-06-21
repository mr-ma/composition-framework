#ifndef COMPOSITION_FRAMEWORK_EDGE_HPP
#define COMPOSITION_FRAMEWORK_EDGE_HPP

#include <cstdint>
#include <string>
#include <iostream>
#include <type_traits>

typedef uintptr_t edge_idx_t;

enum class edge_type {
	UNKNOWN,
	CFG,
	DEPENDENCY,
};


std::ostream &operator<<(std::ostream &os, const edge_type &obj);

struct edge_t {
	explicit edge_t(
			edge_idx_t index = 0,
			const std::string &name = "",
			edge_type type = edge_type::UNKNOWN
	) noexcept;

	edge_idx_t index;
	std::string name;
	edge_type type;
};

std::ostream &operator<<(std::ostream &os, const edge_t &e) noexcept;

bool operator==(const edge_t &lhs, const edge_t &rhs) noexcept;

bool operator!=(const edge_t &lhs, const edge_t &rhs) noexcept;

#endif //COMPOSITION_FRAMEWORK_EDGE_HPP
