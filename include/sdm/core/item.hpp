/**=============================================================================
  Copyright (c) 2016 David Albert
==============================================================================*/
#pragma once

//!
//! \file     item.hpp
//! \author   David Albert
//! \brief    item class
//! \version  1.0
//! \date     11 dec 2020
//!
//! This class provides a generic item.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm
{
    template <class item_t>
    struct Item : public item_t
    {

    public:
        Item(item_t item) : item_t(item)
        {
        }

        friend std::ostream &operator<<(std::ostream &os, const Item<item_t> &it)
        {
            os << it;
            return os;
        }
    };

    template <>
    struct Item<int>
    {
    protected:
        int item_;

    public:
        Item(int item) : item_(item) {}
        operator int()
        {
            return this->item_;
        }
    };

    template <>
    struct Item<unsigned int>
    {
    protected:
        unsigned int item_;

    public:
        Item(unsigned int item) : item_(item) {}
        
        operator unsigned int()
        {
            return this->item_;
        }
    };

    template <>
    struct Item<unsigned short>
    {
    protected:
        unsigned short item_;

    public:
        Item(unsigned short item) : item_(item) {}
        
        operator unsigned short()
        {
            return this->item_;
        }
    };

    template <>
    struct Item<short>
    {
    protected:
        short item_;

    public:
        Item(short item) : item_(item) {}
        
        operator short()
        {
            return this->item_;
        }
    };

    template <>
    struct Item<long>
    {
    protected:
        long item_;

    public:
        Item(long item) : item_(item) {}
        
        operator long()
        {
            return this->item_;
        }
    };
    
    template <>
    struct Item<float>
    {
    protected:
        float item_;

    public:
        Item(float item) : item_(item) {}
        
        operator float()
        {
            return this->item_;
        }
    };

    template <> 
    struct Item<double>
    {
    protected:
        double item_;

    public:
        Item(double item) : item_(item) {}
        operator double()
        {
            return this->item_;
        }
    };

    template <>
    struct Item<bool>
    {
    protected:
        bool item_;

    public:
        Item(bool item) : item_(item) {}
        
        operator bool()
        {
            return this->item_;
        }
    };

    template <>
    struct Item<char>
    {
    protected:
        char item_;

    public:
        Item(char item) : item_(item) {}
        
        operator char()
        {
            return this->item_;
        }
    };

} // namespace sdm
