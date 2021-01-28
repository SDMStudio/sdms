/**
 * @file space.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief abstract space class
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
#include <boost/bimap.hpp>
#include <sdm/types.hpp>
#include <sdm/core/space/space.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
   * @class Space
   * @brief Abstract space class 
   */
    template <typename TSpace = Space>
    class MultiSpace : public Space
    {
    protected:
        std::vector<std::shared_ptr<TSpace>> spaces_;

    public:
        MultiSpace();
        MultiSpace(const std::vector<std::shared_ptr<TSpace>> &);

        bool isDiscrete() const;

        std::vector<number> getDim() const;

        number getNumSpaces() const;

        std::vector<std::shared_ptr<TSpace>> getSpaces() const;
        std::shared_ptr<TSpace> getSpace(number index) const;

        void setSpaces(const std::vector<std::shared_ptr<TSpace>> &);

        std::string str() const;

        MultiSpace &operator=(const MultiSpace &);
        bool operator==(const MultiSpace &) const;
        bool operator!=(const MultiSpace &) const;
        friend std::ostream &operator<<(std::ostream &os, const MultiSpace &sp)
        {
            os << sp.str();
            return os;
        }
    };
} // namespace sdm
#include <sdm/core/space/multi_space.tpp>