#include <sdm/common.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/space/discrete_space.hpp>

namespace sdm
{
    template <typename TItem>
    DiscreteSpace<TItem>::DiscreteSpace() : num_items_(0)
    {
    }

    template <typename TItem>
    DiscreteSpace<TItem>::DiscreteSpace(const std::vector<TItem> &items) : num_items_(items.size()), list_items_(items)
    {
        for (number i = 0; i < this->num_items_; i++)
        {
            this->all_items_.insert(items_bimap_value(i, items[i]));
        }
    }

    template <typename TItem>
    DiscreteSpace<TItem>::DiscreteSpace(std::initializer_list<TItem> vals) : DiscreteSpace(std::vector<TItem>(vals))
    {
    }

    template <typename TItem>
    DiscreteSpace<TItem>::DiscreteSpace(const DiscreteSpace &copy)
        : num_items_(copy.num_items_),
          all_items_(copy.all_items_),
          list_items_(copy.list_items_)
    {
    }

    template <typename TItem>
    template <typename T>
    DiscreteSpace<TItem>::DiscreteSpace(const std::vector<T> &items)
    {
        std::vector<TItem> titems(items.begin(), items.end());
        *this = DiscreteSpace(titems);
    }

    template <typename TItem>
    bool DiscreteSpace<TItem>::isDiscrete() const
    {
        return true;
    }

    template <typename TItem>
    bool DiscreteSpace<TItem>::isStoringItems() const
    {
        return this->store_items_;
    }

    template <typename TItem>
    void DiscreteSpace<TItem>::storeItems(bool store_items)
    {
        this->store_items_ = store_items;
    }

    template <typename TItem>
    bool DiscreteSpace<TItem>::isGenerated()
    {
        return !this->list_items_.empty();
    }

    template <typename TItem>
    TItem DiscreteSpace<TItem>::sample() const
    {
        assert(!this->all_items_.empty());
        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_int_distribution<std::mt19937::result_type> distrib(0, this->all_items_.size() - 1);
        return this->all_items_.left.at(distrib(rng));
    }

    template <typename TItem>
    number DiscreteSpace<TItem>::getNumItems() const
    {
        // std::cout << "getNumItems() this->num_items_ " << this->num_items_ << std::endl;
        return this->num_items_;
    }

    template <typename TItem>
    TItem DiscreteSpace<TItem>::getItem(number index) const
    {
        return this->all_items_.left.at(index);
    }

    template <typename TItem>
    number DiscreteSpace<TItem>::getItemIndex(const TItem &item) const
    {
        return this->all_items_.right.at(item);
    }

    template <typename TItem>
    template <typename T>
    TItem DiscreteSpace<TItem>::getItemAddress(const T &item_value)
    {
        auto end_iter = this->end();
        for (auto iter = this->begin(); !iter->equal(end_iter); iter = iter->next())
        {
            auto item = iter->getCurrent();
            if (*std::static_pointer_cast<T>(item) == item_value)
            {
                return item;
            }
        }
        return nullptr;
    }

    template <typename TItem>
    std::vector<number> DiscreteSpace<TItem>::getDim() const
    {
        return {1};
    }

    template <typename TItem>
    void DiscreteSpace<TItem>::generateItems()
    {
        if (this->isStoringItems())
        {
            this->storeItems(false);
            this->all_items_.clear();
            this->list_items_.clear();

            // Generate joint items and store in containers
            number counter = 0;
            auto end_iter = this->end();
            for (auto iter = this->begin(); !iter->equal(end_iter); iter = iter->next())
            {
                auto item = iter->getCurrent();
                this->all_items_.insert(items_bimap_value(counter, item));
                this->list_items_.push_back(item);
                counter++;
            }
            this->storeItems(true);
        }
    }

    template <typename TItem>
    std::vector<TItem> DiscreteSpace<TItem>::getAll()
    {
        if (!this->store_items_)
        {
            throw sdm::exception::Exception("You are trying to generate all items of space with parameter 'store_items=false'\n#> Use for loop on the space or set parameter store items to 'true'\n");
        }
        else
        {
            if (this->list_items_.empty())
            {
                this->generateItems();
            }
            return this->list_items_;
        }
    }

    template <typename TItem>
    std::string DiscreteSpace<TItem>::str() const
    {
        std::ostringstream res;
        res << "DiscreteSpace(" << this->getNumItems() << ")";
        res << "[";
        for (std::size_t i = 0; i < this->list_items_.size(); ++i)
        {
            res << *this->list_items_[i];
            if (i != this->list_items_.size() - 1)
                res << ", ";
        }
        res << "]";
        return res.str();
    }

    template <typename TItem>
    std::string DiscreteSpace<TItem>::short_str() const
    {
        std::ostringstream res;
        res << "DiscreteSpace(" << this->getNumItems() << ")";
        res << this->list_items_;
        return res.str();
    }

    template <typename TItem>
    bool DiscreteSpace<TItem>::operator==(const DiscreteSpace &sp) const
    {
        if (this->getNumItems() != sp.getNumItems())
        {
            return false;
        }
        else
        {
            for (number id = 0; id < this->getNumItems(); id++)
            {
                if (this->getItem(id) != sp.getItem(id))
                {
                    return false;
                }
            }
            return true;
        }
    }

    template <typename TItem>
    bool DiscreteSpace<TItem>::operator!=(const DiscreteSpace &sp) const
    {
        return !(operator==(sp));
    }

    template <typename TItem>
    int DiscreteSpace<TItem>::find(const TItem &item) const
    {
        auto find = std::find(this->list_items_.begin(), this->list_items_.end(), item);

        if (find != this->list_items_.end())
        {
            return std::distance(this->list_items_.begin(), find);
        }

        return -1;
    }

    template <typename TItem>
    bool DiscreteSpace<TItem>::contains(const TItem &item) const
    {
        return std::find(this->list_items_.begin(), this->list_items_.end(), item) != this->list_items_.end();
    }

    template <typename TItem>
    typename DiscreteSpace<TItem>::iterator_type DiscreteSpace<TItem>::begin()
    {
        return std::make_shared<iterator::SuperIterator<TItem, decltype(list_items_.begin())>>(this->list_items_.begin());
    }

    template <typename TItem>
    typename DiscreteSpace<TItem>::iterator_type DiscreteSpace<TItem>::end()
    {
        return std::make_shared<iterator::SuperIterator<TItem, decltype(list_items_.end())>>(this->list_items_.end());
    }

} // namespace sdm
