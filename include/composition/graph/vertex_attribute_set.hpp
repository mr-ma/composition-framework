#ifndef COMPOSITION_FRAMEWORK_VERTEX_ATTRIBUTE_SET_HPP
#define COMPOSITION_FRAMEWORK_VERTEX_ATTRIBUTE_SET_HPP

#include <unordered_set>
#include <boost/graph/properties.hpp>

namespace composition {
	typedef std::unordered_set<std::string> vertex_attribute_set;
}

namespace boost {
	enum vertex_attribute_set_t { vertex_attribute_set };
	BOOST_INSTALL_PROPERTY(vertex, attribute_set);
}

#endif //COMPOSITION_FRAMEWORK_VERTEX_ATTRIBUTE_SET_HPP
