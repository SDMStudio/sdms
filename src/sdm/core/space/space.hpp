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
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/utils/struct/iterator.hpp>


/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{

  template <typename TItem>
  class DiscreteSpace;

  template <typename TItem>
  class MultiDiscreteSpace;

  /**
   * @class Space
   * 
   * @brief This class is an abstract interface that all spaces should inherite. 
   * 
   * It gives some useful general methods to use generic spaces in your algorithms. 
   
   */
  template <typename TItem>
  class BaseSpace : public std::enable_shared_from_this<BaseSpace<TItem>>
  {
  public:
    using iterator_type = std::shared_ptr<Iterator<TItem>>;

    virtual ~BaseSpace() {}

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

    std::shared_ptr<DiscreteSpace<TItem>> toDiscreteSpace();
    
    std::shared_ptr<MultiDiscreteSpace<TItem>> toMultiDiscreteSpace();

    /**
     * @brief Sample a random item from the space
     */
    virtual std::shared_ptr<TItem> sample() const { throw sdm::exception::Exception("Cannot sample Abstract space !!!"); }

    virtual iterator_type begin() = 0;
    virtual iterator_type end() = 0;

    /**
     * @brief Space as a string
     */
    virtual std::string str() const = 0;

    bool operator==(const BaseSpace &sp) const;
    bool operator!=(const BaseSpace &sp) const;

    friend std::ostream &operator<<(std::ostream &os, const BaseSpace<TItem> &sp)
    {
      os << sp.str();
      return os;
    }
  };

  using Space = BaseSpace<std::shared_ptr<Item>>;
  using StateSpace = BaseSpace<std::shared_ptr<State>>;
  using ObservationSpace = BaseSpace<std::shared_ptr<Observation>>;
  using ActionSpace = BaseSpace<std::shared_ptr<Action>>;

} // namespace sdm
