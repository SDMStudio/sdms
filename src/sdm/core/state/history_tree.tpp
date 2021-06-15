#include <sdm/core/state/history_tree.hpp>

namespace sdm
{
    template <typename T>
    HistoryTree<T>::HistoryTree() : Tree<T>()
    {
    }

    template <typename T>
    HistoryTree<T>::HistoryTree(number max_depth) : Tree<T>(max_depth)
    {
    }

    template <typename T>
    HistoryTree<T>::HistoryTree(std::shared_ptr<HistoryTree<T>> parent, const T &item) : Tree<T>(parent, item)
    {
    }

    template <typename T>
    number HistoryTree<T>::getHorizon() const
    {
        return this->getDepth();
    }

    template <typename T>
    void HistoryTree<T>::expand(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action);
    {
        


        // if (backup && (this->children_.find(data) != this->children_.end()))
        // {
        //     return std::static_pointer_cast<output>(this->getChild(data));
        // }
        // if (backup && (this->getDepth() >= this->getMaxDepth()))
        // {
        //     return this->truncatedExpand<output>(data, backup);
        // }
        // if (backup)
        // {
        //     this->children_.emplace(data, std::make_shared<output>(std::static_pointer_cast<output>(this->shared_from_this()), data));
        //     return std::static_pointer_cast<output>(this->getChild(data));
        // }
        // return std::make_shared<output>(std::static_pointer_cast<output>(this->shared_from_this()), data);
    }

    template <typename T>
    template <typename output>
    std::shared_ptr<output> HistoryTree<T>::truncatedExpand(const T &item, bool backup)
    {
        // std::list<T> items;

        // //<! fill in the vector of observation to simulate
        // items.push_front(item);
        // auto parent = this->shared_from_this();
        // while (items.size() < this->getMaxDepth())
        // {
        //     items.push_front(parent->getData());
        //     parent = std::static_pointer_cast<output>(parent->getParent());
        // }

        // //<! iteratively expands the base_graph
        // auto trace = std::static_pointer_cast<HistoryTree<T>>(parent->getOrigin());

        // for (auto it = items.begin(); it != items.end(); ++it)
        // {
        //     trace = trace->template expand<output>(*it, backup);
        // }

        // return std::static_pointer_cast<output>(trace);
    }

    template <typename T>
    std::string HistoryTree<T>::short_str()
    {
        // std::ostringstream res;
        // std::list<T> list_items;
        // std::shared_ptr<Tree<T>> chistory = this->shared_from_this(), origin = this->getOrigin();
        // while (chistory != origin)
        // {
        //     list_items.push_front(chistory->getData());
        //     chistory = chistory->getParent();
        // }
        // res << '(';
        // for (auto item_it = list_items.begin(); item_it != list_items.end();)
        // {
        //     res << *item_it;
        //     item_it++;
        //     if (item_it != list_items.end())
        //     {
        //         res << ", ";
        //     }
        // }
        // res << ')';
        // return res.str();
    }

    template <typename T>
    std::string HistoryTree<T>::str() const
    {
        std::ostringstream res;
        res << "<history id=\"" << this->shared_from_this() << "\"  horizon=\"" << this->getDepth() << "\" value=\"" << this->short_str() << "\"/>";
        return res.str();
    }

    template <typename T>
    std::shared_ptr<HistoryTree<T>> HistoryTree<T>::getptr()
    {
        return std::static_pointer_cast<HistoryTree<T>>(this->shared_from_this());
    }

    template <typename T>
    std::shared_ptr<HistoryTree<T>> HistoryTree<T>::getParent() const
    {
        // return std::static_pointer_cast<HistoryTree<T>>(Tree<T>::getParent());
    }

    template <typename T>
    std::shared_ptr<HistoryTree<T>> HistoryTree<T>::getOrigin()
    {
        // return std::static_pointer_cast<HistoryTree<T>>(Tree<T>::getOrigin());
    }

    template <typename T>
    std::vector<std::shared_ptr<HistoryTree<T>>> HistoryTree<T>::getChildren() const
    {
        // std::vector<std::shared_ptr<HistoryTree<T>>> vector;
        // for (const auto &children : Tree<T>::getChildren())
        // {
        //     std::cout << "\n children " << *children << std::endl;
        //     vector.push_back(std::static_pointer_cast<HistoryTree<T>>(children));
        // }
        // return vector;
    }

    template <typename T>
    std::shared_ptr<HistoryTree<T>> HistoryTree<T>::getChild(const T &child_item) const
    {
        // return std::static_pointer_cast<HistoryTree<T>>(Tree<T>::getChild(child_item));
    }

    template <typename T>
    template <class Archive>
    void HistoryTree<T>::serialize(Archive &archive, const unsigned int)
    {
        // using boost::serialization::make_nvp;
        // archive &boost::serialization::base_object<Tree<T>>(*this);
    }

} // namespace sdm