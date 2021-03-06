/**
 * @file multi_space.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File implementing MultiSpace class.
 * @version 1.0
 * @date 01/02/2021
 * 
 * @copyright Copyright (c) 2021
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
     * @brief A multi-space is a set a spaces. 
     * 
     * Each space can be of any derived class of Space class.
     * By default Space is set to Space, which means that any space can be used (can be composed of both discrete and continuous spaces). 
     * MultiSpace herites from Space so you can even build hierarchical spaces like MultiSpace of MultiSpace (MultiSpace<MultiSpace<Space>>>).  
     * 
     * @tparam Space 
     */
    class MultiSpace : public Space
    {
    public:
        MultiSpace();
        
        MultiSpace(const std::vector<std::shared_ptr<Space>> &);

        bool isDiscrete() const;

        std::vector<number> getDim() const;

        /**
         * @brief Get the number of sub-space.
         */
        number getNumSpaces() const;

        /**
         * @brief Get all spaces 
         * 
         * @return the list of space pointers 
         */
        std::vector<std::shared_ptr<Space>> getSpaces() const;

        /**
         * @brief Get a specific subspace
         * 
         * @param index the index of the space
         * @return a shared pointer on a specific space 
         */
        std::shared_ptr<Space> getSpace(number index) const;

        /**
         * @brief Change the list of spaces. You may prefer build a new MultiSpace instead of changing values of existing one.
         * 
         */
        void setSpaces(const std::vector<std::shared_ptr<Space>> &);

        std::string str() const;

        MultiSpace &operator=(const MultiSpace &);
        bool operator==(const MultiSpace &) const;
        bool operator!=(const MultiSpace &) const;
        friend std::ostream &operator<<(std::ostream &os, const MultiSpace &sp)
        {
            os << sp.str();
            return os;
        }

    protected:
        /**
         * @brief The list of spaces.
         */
        std::vector<std::shared_ptr<Space>> spaces_;
    };
} // namespace sdm