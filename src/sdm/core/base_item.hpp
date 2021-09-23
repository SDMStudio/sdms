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
     * 
     * Example:
     * 
     * ```cpp
     * BaseItem<char> item('a'), item2('b'); // Instanciate an item stored as a character.   
     * BaseItem<float> float_item(0.0), float_item2(0.1); // Instanciate an item stored as a float. 
     * ```
     * 
     */
    template <class TItem>
    struct BaseItem : public TItem, public Item
    {
    public:
        BaseItem();
        BaseItem(TItem item);
        virtual ~BaseItem();


        bool isBaseItem();
        std::string str() const;

        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &archive, const unsigned int)
        {
            using boost::serialization::make_nvp;

            archive &boost::serialization::base_object<TItem>(*this);
        }

    };

    using StringItem = BaseItem<std::string>;
    using NumberItem = BaseItem<number>;
} // namespace sdm


#include <sdm/core/base_item.tpp>
