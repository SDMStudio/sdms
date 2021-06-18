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
    std::shared_ptr<HistoryTreeInterface> HistoryTree<T>::expand(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action, bool backup )
    {
        if (backup && (this->children_.find(observation) != this->children_.end()))
        {
            return std::static_pointer_cast<HistoryTree<T>>(this->getChild(observation));
        }
        if (backup && (this->getDepth() >= this->getMaxDepth()))
        {
            return this->truncatedExpand(observation,action, backup);
        }
        if (backup)
        {
            this->children_.emplace(observation, std::make_shared<HistoryTree<T>>(this->getptr(), observation));

            return std::static_pointer_cast<HistoryTree<T>>(this->getChild(observation));
        }
        return std::make_shared<HistoryTree<T>>(this->getptr(), observation);

    }

    template <typename T>
    std::shared_ptr<HistoryTreeInterface> HistoryTree<T>::truncatedExpand(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action, bool backup)
    {
        std::list<T> items;

        //<! fill in the vector of observation to simulate
        items.push_front(observation);
        auto parent = this->getptr();
        while (items.size() < this->getMaxDepth())
        {
            items.push_front(parent->getData());
            parent = parent->getParent();
        }

        //<! iteratively expands the base_graph
        auto trace = parent->getOrigin();

        for (auto it = items.begin(); it != items.end(); ++it)
        {
            trace = std::dynamic_pointer_cast<HistoryTree<T>>(trace->expand(*it,action, backup));
        }

        return std::static_pointer_cast<HistoryTree<T>>(trace);
    }

    template <typename T>
    std::string HistoryTree<T>::short_str()
    {
        std::ostringstream res;
        std::list<T> list_items;
        
        std::shared_ptr<Tree<T>> chistory = this->getptr(), origin = this->getOrigin();


        while (chistory != origin)
        {
            list_items.push_front(chistory->getData());
            chistory = chistory->getParent();
        }
        res << '(';
        for (auto item_it = list_items.begin(); item_it != list_items.end();)
        {
            res << *item_it;
            item_it++;
            if (item_it != list_items.end())
            {
                res << ", ";
            }
        }
        res << ')';

        return res.str();
    }

    template <typename T>
    std::string HistoryTree<T>::str() const
    {
        std::ostringstream res;
        res << "<history \"/>";
        return res.str();
    }

    template <typename T>
    std::string HistoryTree<T>::str_not_const()
    {
        std::ostringstream res;
        res << "<history id=\"" << /*this->getptr()<<*/ "\"  horizon=\"" << this->getDepth() << "\" value=\"" << this->short_str() << "\"/>";
        return res.str();
    }

    template <typename T>
    std::shared_ptr<HistoryTree<T>> HistoryTree<T>::getptr()
    {
        return std::dynamic_pointer_cast<HistoryTree<T>>(HistoryTreeInterface::shared_from_this());
    }

    template <typename T>
    std::shared_ptr<HistoryTree<T>> HistoryTree<T>::getParent() const
    {
        return std::static_pointer_cast<HistoryTree<T>>(Tree<T>::getParent());
    }

    template <typename T>
    std::shared_ptr<HistoryTree<T>> HistoryTree<T>::getOrigin()
    {
        return std::static_pointer_cast<HistoryTree<T>>(Tree<T>::getOrigin());
    }

    template <typename T>
    std::vector<std::shared_ptr<HistoryTree<T>>> HistoryTree<T>::getChildren() const
    {
        std::vector<std::shared_ptr<HistoryTree<T>>> vector;
        for (const auto &children : Tree<T>::getChildren())
        {
            std::cout << "\n children " << *children << std::endl;
            vector.push_back(std::static_pointer_cast<HistoryTree<T>>(children));
        }
        return vector;
    }

    template <typename T>
    std::shared_ptr<HistoryTree<T>> HistoryTree<T>::getChild(const T &child_item) const
    {
        return std::static_pointer_cast<HistoryTree<T>>(Tree<T>::getChild(child_item));
    }

    template <typename T>
    template <class Archive>
    void HistoryTree<T>::serialize(Archive &archive, const unsigned int)
    {
        // using boost::serialization::make_nvp;
        // archive &boost::serialization::base_object<Tree<T>>(*this);
    }

} // namespace sdm