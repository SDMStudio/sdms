.. _example:

Examples
==================

Minimal example
-------------------

In this first example, we show how to invoke HSVI algorithm in order to solve a simple problem.

.. code-block:: c++

  #include <sdm/worlds.hpp>
  #include <sdm/algorithms.hpp>

  int main(int argc, char **argv){
    std::shared_ptr<sdm::DecPOMDP> dpomdp = std::make_shared<sdm::DecPOMDP>("path/to/problem.dpomdp");
    auto hsvi = sdm::make<number, number>("tabular_hsvi", dpomdp);
    hsvi->do_solve();
    return 0;
  }
