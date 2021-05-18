#pragma once

#include <iostream>
#include <vector>
#include <boost/serialization/vector.hpp>

#include <sdm/types.hpp>

namespace sdm
{
    // template <typename T>
    // using Vector = std::vector<T>;

    template <typename T>
    std::ostream &operator<<(std::ostream &os, const std::vector<T> &v)
    {
        os << "[";
        for (std::size_t i = 0; i < v.size(); ++i)
        {
            os << v[i];
            if (i != v.size() - 1)
                os << ", ";
        }
        os << "]";
        return os;
    }
}

namespace std
{
  template <typename T>
  struct hash<std::vector<T>>
  {
    typedef std::vector<T> argument_type;
    typedef std::size_t result_type;
    result_type operator()(argument_type const &in) const
    {
      size_t size = in.size();
      size_t seed = 0;
      for (size_t i = 0; i < size; i++)
        //Combine the hash of the current vector with the hashes of the previous ones
        sdm::hash_combine(seed, in[i]);
      return seed;
    }
  };
}

