#include <composition/graph/edge.hpp>

#include <sstream>
#include <composition/graph/graphviz.hpp>

std::ostream &operator<<(std::ostream &os, const edge_type &obj) {
	os << static_cast<std::underlying_type<edge_type>::type>(obj);
	return os;
}

edge_t::edge_t(
		edge_idx_t index,
		const std::string &name,
		edge_type type
) noexcept :
		index{index},
		name{name},
		type{type} {
}

std::ostream &operator<<(std::ostream &os, const edge_t &e) noexcept {
	os << e.index
	   << ","
	   << graphviz_encode(e.name)
	   << ","
	   << e.type;
	return os;
}

bool operator==(const edge_t &lhs, const edge_t &rhs) noexcept {
	return lhs.index == rhs.index;
}

bool operator!=(const edge_t &lhs, const edge_t &rhs) noexcept {
	return !(lhs == rhs);
}