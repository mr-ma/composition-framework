#ifndef COMPOSITION_FRAMEWORK_GRAPHPASS_H
#define COMPOSITION_FRAMEWORK_GRAPHPASS_H

#include <set>
#include <boost/bimap/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <llvm/Pass.h>
#include <composition/graph/ProtectionGraph.hpp>
#include <composition/Manifest.hpp>

namespace composition {
class GraphPass : public llvm::ModulePass {
private:
  std::unique_ptr<ProtectionGraph> Graph{};
public:
  static char ID;
public:
  GraphPass() : ModulePass(ID) {}

  std::vector<std::shared_ptr<Manifest>> SortedManifests();

  void getAnalysisUsage(llvm::AnalysisUsage &usage) const override;

  bool runOnModule(llvm::Module &module) override;

  bool doFinalization(llvm::Module &module) override;

private:
  using ManifestCoverageMap = boost::bimaps::bimap<boost::bimaps::multiset_of<std::shared_ptr<Manifest>>, boost::bimaps::multiset_of<llvm::Instruction*>>;
  using ManifestUndoMap = boost::bimaps::bimap<boost::bimaps::multiset_of<std::shared_ptr<Manifest>>, boost::bimaps::multiset_of<llvm::Value*>>;
  using ProtecteeManifestMap = boost::bimaps::bimap<boost::bimaps::multiset_of<llvm::Instruction*>, std::shared_ptr<Manifest>>;
  using ManifestDependencyMap = boost::bimaps::bimap<boost::bimaps::multiset_of<std::shared_ptr<Manifest>>, boost::bimaps::multiset_of<std::shared_ptr<Manifest>>>;

  ManifestDependencyMap computeManifestDependencies(std::set<std::shared_ptr<Manifest>> manifests);

  template< class MapType >
  void print_map(const MapType & m)
  {
    typedef typename MapType::const_iterator const_iterator;
    for( const_iterator iter = m.begin(), iend = m.end(); iter != iend; ++iter )
    {
      std::cout << iter->first << "-->" << iter->second << "\n";
    }
  }
};
}
#endif //COMPOSITION_FRAMEWORK_GRAPHPASS_H
