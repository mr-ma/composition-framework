#include <composition/graph/dot.hpp>
namespace composition {
bool is_regular_file(const std::string &filename) noexcept {
  std::fstream f;
  f.open(filename.c_str(), std::ios::in);
  return f.is_open();
}

graph_t load_graph_from_dot(const std::string &dot_filename) {
  if (!composition::is_regular_file(dot_filename)) {
    std::stringstream msg;
    msg << __func__ << " : ␣ f i l e ␣ ’ " << dot_filename << " ’ ␣not ␣ found ";
    throw std::invalid_argument(msg.str());
  }

  std::ifstream f(dot_filename.c_str());
  auto g = composition::create_empty_graph();
  boost::dynamic_properties dp(
      boost::ignore_other_properties
  );
  boost::read_graphviz(f, g, dp);
  return g;
}
}