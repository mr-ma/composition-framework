#ifndef COMPOSITION_FRAMEWORK_EDGE_HPP
#define COMPOSITION_FRAMEWORK_EDGE_HPP

#include <boost/graph/properties.hpp>

namespace composition {
	enum edge_type {
		HIERARCHY,
		PROTECTION
	};
}

namespace boost {
	enum edge_type_t { edge_type };
	BOOST_INSTALL_PROPERTY(edge, type);
}

#endif //COMPOSITION_FRAMEWORK_EDGE_HPP
