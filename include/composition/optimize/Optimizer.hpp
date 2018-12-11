#ifndef COMPOSITION_FRAMEWORK_OPTIMIZE_OPTIMIZER_HPP
#define COMPOSITION_FRAMEWORK_OPTIMIZE_OPTIMIZER_HPP

namespace composition::optimize {
/**
 * Abstract class to define a structure for optimizers
 */
class Optimizer {
public:
  virtual ~Optimizer() = default;


  virtual Manifest *optimize() = 0;
};
} 

#endif //COMPOSITION_FRAMEWORK_OPTIMIZE_OPTIMIZER_HPP 
