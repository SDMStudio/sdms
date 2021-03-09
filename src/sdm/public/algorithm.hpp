
#pragma once

namespace sdm
{
  class Algorithm
  {
  public:
    virtual void do_solve() = 0;
    virtual void do_test() = 0;
  };
} // namespace sdm
