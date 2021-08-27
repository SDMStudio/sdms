#pragma once

#include <sdm/types.hpp>
#include <sdm/core/item.hpp>

namespace sdm
{
   
    /**
     * @brief A base class inheriting from the Item interface.
     * @tparam TItem the type of data used for the item.
     * 
     * This class can be used to instantiate an item represented by any type.
     * Example:
     * 
     *      BaseItem<char> item('a'), item2('b'); // Instanciate an item stored as a character.   
     *      BaseItem<float> float_item(0.0), float_item2(0.1); // Instanciate an item stored as a float.   
     * 
     */
    template <class TItem>
    struct BaseItem : public TItem, public Item
    {
    public:
        BaseItem()
        {
        }

        virtual ~BaseItem()
        {
        }

        BaseItem(TItem item) : TItem(item)
        {
        }

        std::string str() const
        {
            std::ostringstream res;
            res << static_cast<TItem>(*this);
            return res.str();
        }
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &archive, const unsigned int)
        {
            using boost::serialization::make_nvp;

            archive &boost::serialization::base_object<TItem>(*this);
        }

        bool isBaseItem(){
            return true;}
    };

    using StringItem = BaseItem<std::string>;
    using NumberItem = BaseItem<number>;

    template <>
    struct BaseItem<int>
    {
    protected:
        int item_;

    public:
        BaseItem() : item_(0)
        {
        }

        BaseItem(int item) : item_(item) {}

        operator int()
        {
            return this->item_;
        }

        std::string str() const
        {
            std::ostringstream res;
            res << this->item_;
            return res.str();
        }
    };

    template <>
    struct BaseItem<unsigned int>
    {
    protected:
        unsigned int item_;

    public:
        BaseItem() : item_(0)
        {
        }

        BaseItem(unsigned int item) : item_(item) {}

        operator unsigned int()
        {
            return this->item_;
        }

        std::string str() const
        {
            std::ostringstream res;
            res << this->item_;
            return res.str();
        }
    };

    template <>
    struct BaseItem<unsigned short>
    {
    protected:
        unsigned short item_;

    public:
        BaseItem() : item_(0)
        {
        }

        BaseItem(unsigned short item) : item_(item) {}

        operator unsigned short()
        {
            return this->item_;
        }

        std::string str() const
        {
            std::ostringstream res;
            res << this->item_;
            return res.str();
        }

        bool operator<(const BaseItem &v2) const
        {
            return this->item_ < v2.item_;
        }
    };

    // template <>
    // struct BaseItem<short>
    // {
    // protected:
    //     short item_;

    // public:
    //     BaseItem() : item_(0)
    //     {
    //     }

    //     BaseItem(short item) : item_(item) {}

    //     operator short()
    //     {
    //         return this->item_;
    //     }

    //     friend std::ostream &operator<<(std::ostream &os, const BaseItem<short> &it)
    //     {
    //         os << it.item_;
    //         return os;
    //     }
    // };

    // template <>
    // struct BaseItem<long>
    // {
    // protected:
    //     long item_;

    // public:
    //     BaseItem() : item_(0)
    //     {
    //     }

    //     BaseItem(long item) : item_(item) {}

    //     operator long()
    //     {
    //         return this->item_;
    //     }

    //     friend std::ostream &operator<<(std::ostream &os, const BaseItem<long> &it)
    //     {
    //         os << it.item_;
    //         return os;
    //     }
    // };

    // template <>
    // struct BaseItem<float>
    // {
    // protected:
    //     float item_;

    // public:
    //     BaseItem() : item_(0)
    //     {
    //     }

    //     BaseItem(float item) : item_(item) {}

    //     operator float()
    //     {
    //         return this->item_;
    //     }

    //     friend std::ostream &operator<<(std::ostream &os, const BaseItem<float> &it)
    //     {
    //         os << it.item_;
    //         return os;
    //     }
    // };

    // template <>
    // struct BaseItem<double>
    // {
    // protected:
    //     double item_;

    // public:
    //     BaseItem() : item_(0)
    //     {
    //     }

    //     BaseItem(double item) : item_(item) {}
    //     operator double()
    //     {
    //         return this->item_;
    //     }

    //     friend std::ostream &operator<<(std::ostream &os, const BaseItem<double> &it)
    //     {
    //         os << it.item_;
    //         return os;
    //     }
    // };

    // template <>
    // struct BaseItem<bool>
    // {
    // protected:
    //     bool item_;

    // public:
    //     BaseItem() : item_(0)
    //     {
    //     }

    //     BaseItem(bool item) : item_(item) {}

    //     operator bool()
    //     {
    //         return this->item_;
    //     }

    //     friend std::ostream &operator<<(std::ostream &os, const BaseItem<bool> &it)
    //     {
    //         os << it.item_;
    //         return os;
    //     }
    // };

    // template <>
    // struct BaseItem<char>
    // {
    // protected:
    //     char item_;

    // public:
    //     BaseItem() : item_('0')
    //     {
    //     }

    //     BaseItem(char item) : item_(item) {}

    //     operator char()
    //     {
    //         return this->item_;
    //     }

    //     friend std::ostream &operator<<(std::ostream &os, const BaseItem<char> &it)
    //     {
    //         os << it.item_;
    //         return os;
    //     }
    // };

} // namespace sdm

namespace std
{

    template <typename TState>
    struct hash<sdm::BaseItem<TState>>
    {
        typedef sdm::BaseItem<TState> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            return std::hash<TState>()(in);
        }
    };
}
