#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/symbolic/SymbolicEliminationTree.h>
#include <gtsam/symbolic/SymbolicJunctionTree.h>

#include <boost/assign/list_of.hpp>
using namespace boost::assign;

#include "symbolicExampleGraphs.h"

using namespace gtsam;
using namespace std;

int main(int argc, char** argv) {

Ordering order; order += 0, 1, 2, 3;

  SymbolicJunctionTree actual(SymbolicEliminationTree(simpleChain, order));

   actual.print();
  SymbolicJunctionTree::Node::Keys
    frontal1 = list_of(2)(3),
    frontal2 = list_of(0)(1),
    sep1, sep2 = list_of(2);


}