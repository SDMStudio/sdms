/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>

#include <sdm/types.hpp>
#include <sdm/core/space/space.hpp>

//!
//! \file     discrete_space.hpp
//! \author   David Albert
//! \brief    discrete space class
//! \version  1.0
//! \date     24 novembre 2020
//!
//! This class provide a way to instantiate discrete space.

//! \namespace  sdm
//! \brief Namespace grouping all tools required for sequential decision making.
namespace sdm
{
    //! \class  Space  space.hpp
    class DiscreteSpace : public Space
    {
    protected:
        //! \brief number of possible elements in the space (ex: [0, 12] --> 13 elements)
        number num_elements_;

        //! \brief map of names.
        boost::bimaps::bimap<std::string, sdm::size_t> names_bimap_;

    public:
        //! \fn     DiscreteSpace()
        //! \brief  instantiate a default discrete space (DiscreteSpace class)
        //! \return instance of DiscreteSpace
        DiscreteSpace();

        //! \fn     DiscreteSpace(number num_elements)
        //! \brief  instantiate a discrete space (DiscreteSpace class)
        //! \param  num_elements the number of possible elements.
        //! \return instance of DiscreteSpace
        DiscreteSpace(number);

        //! \fn     DiscreteSpace(const std::vector<std::string> & e_names)
        //! \brief  instantiate a discrete space (DiscreteSpace class)
        //! \param  e_names a list of element names.
        //! \return instance of DiscreteSpace
        DiscreteSpace(const std::vector<std::string> &);

        //! \fn     number sample()
        //! \brief  Sample an element from the space
        //! \return uniformly sampled an element in the space
        number sample() const;

        //! \fn       number getLength()
        //! \brief    Returns the space length
        number getLength() const;

        //! \fn       number getNumElements()
        //! \brief    Getter for num_elements_ parameter
        number getNumElements() const;

        std::vector<number> getDim() const;

        //! \fn       number getElementIndex(const std::string&)
        //! \param    e_name the element name
        //! \brief    Returns the index of the element (from name)
        number getElementIndex(const std::string &) const;

        //! \fn       std::string getElementName(number)
        //! \param    index index of the element
        //! \brief    Returns the name associated with the index
        std::string getElementName(number) const;

        //! \fn       void setNumElements(number)
        //! \brief    Sets the number of possible elements
        void setNumElements(number);

        //! \fn       void setElementsNames(std::vector<std::string>&)
        //! \param    e_names a list of element names
        //! \brief    Sets the names of elements.

        void setElementsNames(const std::vector<std::string> &s);

        std::string str() const;
        
        bool isDiscrete() const
        {
            return true;
        }

        DiscreteSpace &operator=(const DiscreteSpace &);
        bool operator==(const DiscreteSpace &) const;
        bool operator!=(const DiscreteSpace &) const;
        friend std::ostream &operator<<(std::ostream &os, const DiscreteSpace &sp);
    };
} // namespace sdm
