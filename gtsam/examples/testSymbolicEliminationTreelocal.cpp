
#include <vector>
#include <boost/make_shared.hpp>
#include <boost/assign/list_of.hpp>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/symbolic/SymbolicEliminationTree.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/symbolic/SymbolicFactor.h>
#include <gtsam/symbolic/SymbolicConditional.h>
#include <gtsam/symbolic/SymbolicBayesTree.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <boost/assign/list_of.hpp>
#include "symbolicExampleGraphs.h"
#include <gtsam/symbolic/SymbolicJunctionTree.h>

using boost::assign::list_of;

using namespace std;
using namespace gtsam;
class EliminationTreeTester {
public:
  // build hardcoded tree
  static SymbolicEliminationTree buildHardcodedTree(const SymbolicFactorGraph& fg) {

    SymbolicEliminationTree::sharedNode leaf0(new SymbolicEliminationTree::Node);
    leaf0->key = 0;
    leaf0->factors.push_back(fg[0]);
    leaf0->factors.push_back(fg[1]);

    SymbolicEliminationTree::sharedNode node1(new SymbolicEliminationTree::Node);
    node1->key = 1;
    node1->factors.push_back(fg[2]);
    node1->children.push_back(leaf0);

    SymbolicEliminationTree::sharedNode node2(new SymbolicEliminationTree::Node);
    node2->key = 2;
    node2->factors.push_back(fg[3]);
    node2->children.push_back(node1);

    SymbolicEliminationTree::sharedNode leaf3(new SymbolicEliminationTree::Node);
    leaf3->key = 3;
    leaf3->factors.push_back(fg[4]);

    SymbolicEliminationTree::sharedNode root(new SymbolicEliminationTree::Node);
    root->key = 4;
    root->children.push_back(leaf3);
    root->children.push_back(node2);

    SymbolicEliminationTree tree;
    tree.roots_.push_back(root);
    return tree;
  }

  template<typename ROOTS>
  static SymbolicEliminationTree MakeTree(const ROOTS& roots)
  {
    SymbolicEliminationTree et;
    et.roots_.assign(roots.begin(), roots.end());
    return et;
  }
};

template<typename FACTORS>
static SymbolicEliminationTree::sharedNode MakeNode(Key key, const FACTORS& factors)
{
  SymbolicEliminationTree::sharedNode node = boost::make_shared<SymbolicEliminationTree::Node>();
  node->key = key;
  SymbolicFactorGraph factorsAsGraph = factors;
  node->factors.assign(factorsAsGraph.begin(), factorsAsGraph.end());
  return node;
}

template<typename FACTORS, typename CHILDREN>
static SymbolicEliminationTree::sharedNode MakeNode(Key key, const FACTORS& factors, const CHILDREN& children)
{
  SymbolicEliminationTree::sharedNode node = boost::make_shared<SymbolicEliminationTree::Node>();
  node->key = key;
  SymbolicFactorGraph factorsAsGraph = factors;
  node->factors.assign(factorsAsGraph.begin(), factorsAsGraph.end());
  node->children.assign(children.begin(), children.end());
  return node;
}


void test1() {
  SymbolicFactorGraph graph;
  graph += SymbolicFactor(symbol_shorthand::X(1), symbol_shorthand::L(1));
  graph += SymbolicFactor(symbol_shorthand::X(1), symbol_shorthand::X(2));
  graph += SymbolicFactor(symbol_shorthand::X(2), symbol_shorthand::L(1));
  graph += SymbolicFactor(symbol_shorthand::X(2), symbol_shorthand::X(3));
  graph += SymbolicFactor(symbol_shorthand::X(3), symbol_shorthand::X(4));
  graph += SymbolicFactor(symbol_shorthand::X(4), symbol_shorthand::L(2));
  graph += SymbolicFactor(symbol_shorthand::X(4), symbol_shorthand::X(5));
  graph += SymbolicFactor(symbol_shorthand::L(2), symbol_shorthand::X(5));
  graph += SymbolicFactor(symbol_shorthand::X(4), symbol_shorthand::L(3));
  graph += SymbolicFactor(symbol_shorthand::X(5), symbol_shorthand::L(3));
  
  SymbolicEliminationTree expected = EliminationTreeTester::MakeTree(list_of
    (MakeNode(symbol_shorthand::X(3), SymbolicFactorGraph(), list_of
      (MakeNode(symbol_shorthand::X(2), list_of(SymbolicFactor(symbol_shorthand::X(2), symbol_shorthand::X(3))), list_of
        (MakeNode(symbol_shorthand::L(1), list_of(SymbolicFactor(symbol_shorthand::X(2), symbol_shorthand::L(1))), list_of
          (MakeNode(symbol_shorthand::X(1), list_of(SymbolicFactor(symbol_shorthand::X(1), symbol_shorthand::L(1))) (SymbolicFactor(symbol_shorthand::X(1), symbol_shorthand::X(2)))))))))
      (MakeNode(symbol_shorthand::X(4), list_of(SymbolicFactor(symbol_shorthand::X(3), symbol_shorthand::X(4))), list_of
        (MakeNode(symbol_shorthand::L(2), list_of(SymbolicFactor(symbol_shorthand::X(4), symbol_shorthand::L(2))), list_of
          (MakeNode(symbol_shorthand::X(5), list_of(SymbolicFactor(symbol_shorthand::X(4), symbol_shorthand::X(5))) (SymbolicFactor(symbol_shorthand::L(2), symbol_shorthand::X(5))), list_of
            (MakeNode(symbol_shorthand::L(3), list_of(SymbolicFactor(symbol_shorthand::X(4), symbol_shorthand::L(3))) (SymbolicFactor(symbol_shorthand::X(5), symbol_shorthand::L(3))))))))))))));

  std::cout << "Priting elimination tree " << std::endl;
  Ordering order = list_of(symbol_shorthand::X(1)) (symbol_shorthand::L(3)) (symbol_shorthand::L(1)) (symbol_shorthand::X(5)) (symbol_shorthand::X(2)) (symbol_shorthand::L(2)) (symbol_shorthand::X(4)) (symbol_shorthand::X(3));

  SymbolicEliminationTree actual(graph, order);
  actual.print();

  auto etree = SymbolicJunctionTree(actual);
  std::cout << "Symbolic junction tree " << std::endl;
  etree.print();

}

void test2() {
  SymbolicFactorGraph graph;
  graph += SymbolicFactor(symbol_shorthand::X(3), symbol_shorthand::X(4));
  graph += SymbolicFactor(symbol_shorthand::X(4), symbol_shorthand::X(5));
  graph += SymbolicFactor(symbol_shorthand::X(4), symbol_shorthand::L(1));
  graph += SymbolicFactor(symbol_shorthand::X(5), symbol_shorthand::L(1));
  graph += SymbolicFactor(symbol_shorthand::X(3));
  graph += SymbolicFactor(symbol_shorthand::X(3));
  
  SymbolicEliminationTree expected = EliminationTreeTester::MakeTree(list_of
    (MakeNode(symbol_shorthand::X(3), SymbolicFactorGraph(), list_of
      (MakeNode(symbol_shorthand::X(2), list_of(SymbolicFactor(symbol_shorthand::X(2), symbol_shorthand::X(3))), list_of
        (MakeNode(symbol_shorthand::L(1), list_of(SymbolicFactor(symbol_shorthand::X(2), symbol_shorthand::L(1))), list_of
          (MakeNode(symbol_shorthand::X(1), list_of(SymbolicFactor(symbol_shorthand::X(1), symbol_shorthand::L(1))) (SymbolicFactor(symbol_shorthand::X(1), symbol_shorthand::X(2)))))))))
      (MakeNode(symbol_shorthand::X(4), list_of(SymbolicFactor(symbol_shorthand::X(3), symbol_shorthand::X(4))), list_of
        (MakeNode(symbol_shorthand::L(2), list_of(SymbolicFactor(symbol_shorthand::X(4), symbol_shorthand::L(2))), list_of
          (MakeNode(symbol_shorthand::X(5), list_of(SymbolicFactor(symbol_shorthand::X(4), symbol_shorthand::X(5))) (SymbolicFactor(symbol_shorthand::L(2), symbol_shorthand::X(5))), list_of
            (MakeNode(symbol_shorthand::L(3), list_of(SymbolicFactor(symbol_shorthand::X(4), symbol_shorthand::L(3))) (SymbolicFactor(symbol_shorthand::X(5), symbol_shorthand::L(3))))))))))))));

  std::cout << "Priting elimination tree " << std::endl;
  Ordering order = list_of(symbol_shorthand::X(3)) (symbol_shorthand::L(1)) (symbol_shorthand::X(4)) (symbol_shorthand::X(5));

  SymbolicEliminationTree actual(graph, order);
  actual.print();

  auto etree = SymbolicJunctionTree(actual);
  std::cout << "Symbolic junction tree " << std::endl;
  etree.print();
}

int main(int argc, char** argv) {
  test2();

}