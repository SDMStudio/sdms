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
#include <sdm/exception.hpp>
#include <sdm/core/item.hpp>
#include <sdm/utils/struct/iterator.hpp>

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
  class Space
  {
  public:
    using iterator_type = std::shared_ptr<ItemIterator>;

    virtual ~Space() {}

    /**
     * @brief Check if the space is discrete.
     */
    virtual bool isDiscrete() const = 0;

    /**
     * @brief Check if the space is continous.
     */
    bool isContinuous() const;

    /**
     * @brief Get the dimension of the space.
     */
    virtual std::vector<number> getDim() const = 0;

    /**
     * @brief Sample a random item from the space
     */
    virtual std::shared_ptr<Item> sample() const { throw sdm::exception::Exception("Cannot sample Abstract space !!!"); }

    virtual iterator_type begin() = 0;
    virtual iterator_type end() = 0;

    /**
     * @brief Space as a string
     */
    virtual std::string str() const = 0;

    bool operator==(const Space &sp) const;
    bool operator!=(const Space &sp) const;

    friend std::ostream &
    operator<<(std::ostream &os, const Space &sp)
    {
      os << sp.str();
      return os;
    }
  };
} // namespace sdm
