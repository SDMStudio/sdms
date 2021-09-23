#include <sdm/core/state/history_tree.hpp>

namespace sdm
{
    HistoryTree::HistoryTree() : Tree<std::shared_ptr<Observation>>()
    {
    }

    HistoryTree::HistoryTree(number max_depth) : Tree<std::shared_ptr<Observation>>(max_depth)
    {
    }

    HistoryTree::HistoryTree(std::shared_ptr<HistoryTree> parent, const std::shared_ptr<Observation> &item) : Tree<std::shared_ptr<Observation>>(parent, item)
    {
    }

    std::shared_ptr<HistoryInterface> HistoryTree::getPreviousHistory()
    {
        return this->getParent();
    }

    std::shared_ptr<Observation> HistoryTree::getLastObservation()
    {
        return this->getData();
    }

    number HistoryTree::getHorizon() const
    {
        return this->getDepth();
    }

    // template <typename output>
    // std::shared_ptr<output> HistoryTree::expand(const std::shared_ptr<Observation> &observation, bool backup)
    // {
    //     if (backup && (this->children_.find(observation) != this->children_.end()))
    //     {
    //         return std::dynamic_pointer_cast<output>(this->getChild(observation));
    //     }
    //     if (backup && (this->getDepth() >= this->getMaxDepth()))
    //     {
    //         return this->truncatedExpand<output>(observation, backup);
    //     }
    //     if (backup)
    //     {
    //         this->children_.emplace(observation, std::make_shared<output>(this->getptr(), observation));

    //         return std::dynamic_pointer_cast<output>(this->getChild(observation));
    //     }
    //     return std::make_shared<output>(this->getptr(), observation);
    // }

    std::shared_ptr<HistoryInterface> HistoryTree::expand(const std::shared_ptr<Observation> &observation, bool backup)
    {
        return this->expand<HistoryTree>(observation, backup);
    }

    // template <typename output>
    // std::shared_ptr<output> HistoryTree::truncatedExpand(const std::shared_ptr<Observation> &observation, bool backup)
    // {
    //     std::list<std::shared_ptr<Observation>> items;

    //     //<! fill in the vector of observation to simulate
    //     items.push_front(observation);
    //     auto parent = this->getptr();
    //     while (items.size() < this->getMaxDepth())
    //     {
    //         items.push_front(parent->getData());
    //         parent = parent->getParent();
    //     }

    //     //<! iteratively expands the base_graph
    //     auto trace = parent->getOrigin();

    //     for (auto it = items.begin(); it != items.end(); ++it)
    //     {
    //         trace = std::dynamic_pointer_cast<output>(trace->expand(*it, backup));
    //     }

    //     return std::dynamic_pointer_cast<output>(trace);
    // }

    std::string HistoryTree::short_str() const
    {
        std::ostringstream res;

        if (this->isOrigin())
        {
            res << "()";
        }
        else
        {
            std::list<std::shared_ptr<Observation>> list_items;

            list_items.push_front(this->getData());

            auto chistory = this->getParent();
            while (chistory != this->origin_.lock())
            {
                list_items.push_front(chistory->getData());
                chistory = chistory->getParent();
            }
            res << '(';
            for (auto item_it = list_items.begin(); item_it != list_items.end();)
            {
                res << (*item_it)->str();
                item_it++;
                if (item_it != list_items.end())
                {
                    res << ", ";
                }
            }
            res << ')';
        }

        return res.str();
    }

    std::string HistoryTree::str() const
    {
        std::ostringstream res;
        res << "<history id=\"" << this << "\"  horizon=\"" << this->getDepth() << "\" value=\"" << this->short_str() << "\"/>";
        return res.str();
    }

    std::shared_ptr<Item> HistoryTree::getPointer()
    {
        return this->getptr();
    }

    std::shared_ptr<HistoryTree> HistoryTree::getptr()
    {
        return Tree<std::shared_ptr<Observation>>::downcasted_shared_from_this<HistoryTree>();
    }

    std::shared_ptr<HistoryTree> HistoryTree::getParent() const
    {
        return std::static_pointer_cast<HistoryTree>(Tree<std::shared_ptr<Observation>>::getParent());
    }

    std::shared_ptr<HistoryTree> HistoryTree::getOrigin()
    {
        return std::static_pointer_cast<HistoryTree>(Tree<std::shared_ptr<Observation>>::getOrigin());
    }

    std::vector<std::shared_ptr<HistoryTree>> HistoryTree::getChildren() const
    {
        std::vector<std::shared_ptr<HistoryTree>> vector;
        for (const auto &children : Tree<std::shared_ptr<Observation>>::getChildren())
        {
            std::cout << "\n children " << *children << std::endl;
            vector.push_back(std::static_pointer_cast<HistoryTree>(children));
        }
        return vector;
    }

    std::shared_ptr<HistoryTree> HistoryTree::getChild(const std::shared_ptr<Observation> &child_item) const
    {
        return std::static_pointer_cast<HistoryTree>(Tree<std::shared_ptr<Observation>>::getChild(child_item));
    }

    template <class Archive>
    void HistoryTree::serialize(Archive &, const unsigned int)
    {
        // using boost::serialization::make_nvp;
        // archive &boost::serialization::base_object<Tree<std::shared_ptr<Observation>>>(*this);
    }

    const std::shared_ptr<Observation> &HistoryTree::getData() const
    {
        return Tree<std::shared_ptr<Observation>>::getData();
    }

} // namespace sdm