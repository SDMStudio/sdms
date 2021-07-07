#include <sdm/common.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/space/discrete_space.hpp>

namespace sdm
{
    DiscreteSpace::DiscreteSpace() : num_items_(0) 
    {
        // std::cout << "DiscreteSpace() a" << std::endl;
        // std::cout << "this->num_items_ " << this->num_items_ << std::endl;
    }

    DiscreteSpace::DiscreteSpace(const std::vector<std::shared_ptr<Item>> &items) : num_items_(items.size()), list_items_(items)
    {
        // std::cout << "DiscreteSpace() b" << std::endl;
        // std::cout << "this->num_items_ " << this->num_items_ << std::endl;
        for (number i = 0; i < this->num_items_; i++)
        {
            // std::cout << "i " << i << std::endl;
            this->all_items_.insert(items_bimap_value(i, items[i]));
        }
    }

    // template <typename T>
    // DiscreteSpace::DiscreteSpace(const std::vector<T> &items)
    // {
    //     std::vector<std::shared_ptr<Item>> titems(items.begin(), items.end());
    //     *this = DiscreteSpace(titems);
    // }

    DiscreteSpace::DiscreteSpace(std::initializer_list<std::shared_ptr<Item>> vals) : DiscreteSpace(std::vector<std::shared_ptr<Item>>(vals))
    {
        // std::cout << "DiscreteSpace() c" << std::endl;
    }

    DiscreteSpace::DiscreteSpace(const DiscreteSpace &copy)
        : num_items_(copy.num_items_),
          all_items_(copy.all_items_),
          list_items_(copy.list_items_)
    {
        // std::cout << "DiscreteSpace() d" << std::endl;
    }

    bool DiscreteSpace::isDiscrete() const
    {
        return true;
    }

    bool DiscreteSpace::isStoringItems() const
    {
        return this->store_items_;
    }

    void DiscreteSpace::storeItems(bool store_items)
    {
        this->store_items_ = store_items;
    }

    bool DiscreteSpace::isGenerated()
    {
        return !this->list_items_.empty();
    }

    std::shared_ptr<Item> DiscreteSpace::sample() const
    {
        assert(!this->all_items_.empty());
        std::uniform_int_distribution<int> distr(0, this->all_items_.size() - 1);
        return this->all_items_.left.at(distr(common::global_urng()));
    }
    
    number DiscreteSpace::getNumItems() const
    {
        // std::cout << "getNumItems() this->num_items_ " << this->num_items_ << std::endl;
        return this->num_items_;
    }

    std::shared_ptr<Item> DiscreteSpace::getItem(number index) const
    {
        return this->all_items_.left.at(index);
    }

    number DiscreteSpace::getItemIndex(const std::shared_ptr<Item> &item) const
    {
        return this->all_items_.right.at(item);
    }

    // template <typename T>
    // std::shared_ptr<Item> DiscreteSpace::getItemAddress(const T &item_value)
    // {
    //     for (const auto &item : *this)
    //     {
    //         if (item_value == *std::static_pointer_cast<T>(item))
    //         {
    //             return item;
    //         }
    //     }
    //     return nullptr;
    // }

    std::vector<number> DiscreteSpace::getDim() const
    {
        return {1};
    }

    void DiscreteSpace::generateItems()
    {
        if (this->isStoringItems())
        {
            this->storeItems(false);
            this->all_items_.clear();
            this->list_items_.clear();

            // Generate joint items and store in containers
            number counter = 0;
            for (const auto &v : *this)
            {
                this->all_items_.insert(items_bimap_value(counter, v));
                this->list_items_.push_back(v);
                counter++;
            }
            this->storeItems(true);
        }
    }

    std::vector<std::shared_ptr<Item>> DiscreteSpace::getAll()
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

    std::string DiscreteSpace::str() const
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

    std::string DiscreteSpace::short_str() const
    {
        std::ostringstream res;
        res << "DiscreteSpace(" << this->getNumItems() << ")";
        res << this->list_items_;
        return res.str();
    }

    bool DiscreteSpace::operator==(const DiscreteSpace &sp) const
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

    bool DiscreteSpace::operator!=(const DiscreteSpace &sp) const
    {
        return !(operator==(sp));
    }

    int DiscreteSpace::find(const std::shared_ptr<Item> &item) const
    {
        auto find = std::find(this->list_items_.begin(), this->list_items_.end(), item);

        if(find != this->list_items_.end())
        {
            return std::distance(this->list_items_.begin(),find);
        }

        return -1;
    }

    bool DiscreteSpace::contains(const std::shared_ptr<Item> &item) const
    {
        return std::find(this->list_items_.begin(), this->list_items_.end(), item) != this->list_items_.end() ? true : false;
    }

    DiscreteSpace::iterator_type DiscreteSpace::begin()
    {
        return std::make_shared<iterator::SuperIterator<std::shared_ptr<Item>, decltype(list_items_.begin())>>(this->list_items_.begin());
    }

    DiscreteSpace::iterator_type DiscreteSpace::end()
    {
        return std::make_shared<iterator::SuperIterator<std::shared_ptr<Item>, decltype(list_items_.end())>>(this->list_items_.end());
    }

} // namespace sdm
