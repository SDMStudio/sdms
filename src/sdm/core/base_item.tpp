#include <sdm/core/base_item.hpp>

#define DEFINE_BASE_ITEM(CLASS)               \
    template <>                               \
    struct BaseItem<CLASS> : public Item      \
    {                                         \
    protected:                                \
        CLASS item_;                          \
                                              \
    public:                                   \
        BaseItem() : item_(0)                 \
        {                                     \
        }                                     \
                                              \
        BaseItem(CLASS item) : item_(item) {} \
                                              \
        operator CLASS()                      \
        {                                     \
            return this->item_;               \
        }                                     \
                                              \
        std::string str() const               \
        {                                     \
            std::ostringstream res;           \
            res << this->item_;               \
            return res.str();                 \
        }                                     \
    };

namespace sdm
{
    DEFINE_BASE_ITEM(int);
    DEFINE_BASE_ITEM(unsigned int);
    DEFINE_BASE_ITEM(short);
    DEFINE_BASE_ITEM(unsigned short);
    DEFINE_BASE_ITEM(float);
    DEFINE_BASE_ITEM(double);
    DEFINE_BASE_ITEM(bool);


    template <class TItem>
    BaseItem<TItem>::BaseItem()
    {
    }

    template <class TItem>
    BaseItem<TItem>::BaseItem(TItem item) : TItem(item)
    {
    }

    template <class TItem>
    BaseItem<TItem>::~BaseItem()
    {
    }

    template <class TItem>
    std::string BaseItem<TItem>::str() const
    {
        std::ostringstream res;
        res << static_cast<TItem>(*this);
        return res.str();
    }

    template <class TItem>
    bool BaseItem<TItem>::isBaseItem()
    {
        return true;
    }
}

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