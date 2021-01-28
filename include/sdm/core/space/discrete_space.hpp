
#pragma once

#include <vector>

#include <sdm/core/space/space.hpp>
#include <sdm/utils/decision_rules/variations.hpp>
#include <sdm/types.hpp>

//!
//! \file     D_space.hpp
//! \author   David Albert
//! \brief    D space class
//! \version  1.0
//! \date     24 novembre 2020
//!
//! This class provide a way to instantiate D space.

//! \namespace  sdm
//! \brief Namespace grouping all tools required for sequential decision making.
namespace sdm
{
    /**
     * @brief 
     * 
     * @tparam TItem Base item (string, int, short, double, etc)
     */
    template <typename TItem>
    class DSpace : public Space
    {
    protected:
        //! \brief number of possible elements in the space (ex: [0, 12] --> 13 elements)
        number num_elements_;

        //! \brief number of possible elements in the space (ex: [0, 12] --> 13 elements)
        std::vector<TItem> all_items_;

        //! \brief map of names.
        // boost::bimaps::bimap<sdm::size_t, TItem> names_bimap_;

    public:

        //! \fn     DSpace()
        //! \brief  instantiate a default D space (DSpace class)
        //! \return instance of DSpace
        DSpace();

        //! \fn     DSpace(number num_elements)
        //! \brief  instantiate a D space (DSpace class)
        //! \param  num_elements the number of possible elements.
        //! \return instance of DSpace
        DSpace(number num_elements);

        //! \fn     DSpace(const std::vector<std::string> & items)
        //! \brief  instantiate a D space (DSpace class)
        //! \param  items a list of element names.
        //! \return instance of DSpace
        DSpace(const std::vector<TItem> &items);

        DSpace(const DSpace<TItem> &copy);

        DSpace(std::initializer_list<TItem> vals) : all_items_(vals)
        {
            this->num_elements_ = all_items_.size();
        }

        bool isDiscrete() const;

        //! \fn     number sample()
        //! \brief  Sample an element from the space
        //! \return uniformly sampled an element in the space
        TItem sample() const;

        //! \fn       number getLength()
        //! \brief    Returns the space length
        number getLength() const;

        //! \fn       number getNumElements()
        //! \brief    Getter for num_elements_ parameter
        number getNumElements() const;

        std::vector<TItem> getAll();

        std::vector<number> getDim() const;

        std::string str() const;

        //! \fn       number getElementIndex(const std::string&)
        //! \param    e_name the element name
        //! \brief    Returns the index of the element (from name)
        number getElementIndex(const TItem &item) const;

        //! \fn       std::string getElementName(number)
        //! \param    index index of the element
        //! \brief    Returns the name associated with the index
        TItem getElement(number index) const;

        void setNumElements(number num_elements);

        DSpace &operator=(const DSpace &sp);

        bool operator==(const DSpace &sp) const;

        bool operator!=(const DSpace &sp) const;

        friend std::ostream &operator<<(std::ostream &os, const DSpace &sp)
        {
            os << sp.str();
            return os;
        }

        iterator begin() { return this->all_items_.begin(); }
        iterator end() { return this->all_items_.end(); }
    };
} // namespace sdm

#include <sdm/core/space/discrete_space.tpp>
