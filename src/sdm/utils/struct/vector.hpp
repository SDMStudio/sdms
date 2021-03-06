#pragma once

#include <iostream>
#include <boost/serialization/vector.hpp>

#include <sdm/types.hpp>


namespace std
{
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

  template <typename T>
  struct hash<std::vector<T>>
  {
    typedef std::vector<T> argument_type;
    typedef std::size_t result_type;
    result_type operator()(argument_type const &in) const
    {
      size_t size = in.size();
      size_t seed = 0;
      //Combine the hash of the current vector with the hashes of the previous ones
      for (size_t i = 0; i < size; i++)
      {
        sdm::hash_combine(seed, in[i]);
      }
      return seed;
    }
  };
}
