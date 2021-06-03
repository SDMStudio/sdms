/**
 * @file space.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File for Space class
 * @version 0.1
 * @date 17/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 * This is an abstract interface for Spaces.
 * 
 */
#pragma once

#include <vector>
#include <sdm/types.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
  /**
   * @class Space
   * @brief This class is an abstract interface that all spaces should inherite. It gives some useful general methods to use generic spaces in your algorithms. 
   */
  template <typename TItem>
  class Space
  {
  public:
    virtual ~Space() {}

    /**
     * @brief Check if the space is discrete.
     */
    virtual bool isDiscrete() const = 0;

    /**
     * @brief Check if the space is continous.
     */
    bool isContinuous() const
    {
      return !(this->isDiscrete());
    }

    /**
     * @brief Get the dimension of the space.
     */
    virtual std::vector<number> getDim() const = 0;

    virtual std::vector<TItem> getAll() const = 0;

    // virtual TItem begin() const = 0;

    // virtual TItem next() const = 0;

    // virtual TItem end() const = 0;

    /**
     * @brief Space as a string
     */
    virtual std::string str() const = 0;

    bool operator==(const Space &sp) const
    {
      if (this->str() == sp.str())
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    bool operator!=(const Space &sp) const
    {
      return !(this->operator==(sp));
    }

    friend std::ostream &operator<<(std::ostream &os, const Space &sp)
    {
      os << sp.str();
      return os;
    }
  };
} // namespace sdm
