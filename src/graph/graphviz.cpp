#include <composition/graph/graphviz.hpp>

#include <boost/algorithm/string/replace.hpp>

std::string graphviz_encode(std::string s) noexcept {
	boost::algorithm::replace_all(s, ",", "$$$COMMA$$$");
	boost::algorithm::replace_all(s, " ", "$$$SPACE$$$");
	boost::algorithm::replace_all(s, "\"", "$$$QUOTE$$$");
	return s;
}

std::string grapviz_decode(std::string s) noexcept {
	boost::algorithm::replace_all(s, "$$$COMMA$$$", " , ");
	boost::algorithm::replace_all(s, "$$$SPACE$$$", " ");
	boost::algorithm::replace_all(s, "$$$QUOTE$$$", "\"");
	return s;
}
