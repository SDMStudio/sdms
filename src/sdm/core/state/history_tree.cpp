#include <sdm/core/state/history_tree.hpp>

namespace sdm
{
    HistoryTree::HistoryTree() : Tree<Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>>>()
    {
    }

    HistoryTree::HistoryTree(number max_depth) : Tree<Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>>>(max_depth)
    {
    }

    HistoryTree::HistoryTree(std::shared_ptr<HistoryTree> parent, const Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>> &item) : Tree<Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>>>(parent, item)
    {
    }

    std::shared_ptr<HistoryInterface> HistoryTree::getPreviousHistory()
    {
        return this->getParent();
    }

    std::shared_ptr<Observation> HistoryTree::getLastObservation()
    {
        return this->getData().first;
    }

    number HistoryTree::getHorizon() const
    {
        return this->getDepth();
    }

    std::shared_ptr<HistoryInterface> HistoryTree::expand(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action, bool backup)
    {
        return this->expand<HistoryTree>(observation, action, backup);
    }


    std::shared_ptr<HistoryTree> HistoryTree::expandHistoryTree(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action, bool backup)
    {
        return this->expand<HistoryTree>(observation, action, backup);
    }

    std::string HistoryTree::short_str() const
    {
        std::ostringstream res;

        if (this->isOrigin())
        {
            res << "()";
        }
        else
        {
            std::list<Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>>> list_items;

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
                res << item_it->first->str();
                if (item_it->second != nullptr)
                {
                    res << "-" << item_it->second->str();
                }
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

    std::shared_ptr<HistoryTree> HistoryTree::getptr()
    {
        return std::dynamic_pointer_cast<HistoryTree>(Item::getPointer());
        // return Tree<Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>>>::downcasted_shared_from_this<HistoryTree>();
    }

    std::shared_ptr<HistoryTree> HistoryTree::getParent() const
    {
        return std::static_pointer_cast<HistoryTree>(Tree<Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>>>::getParent());
    }

    std::shared_ptr<HistoryTree> HistoryTree::getOrigin()
    {
        return std::static_pointer_cast<HistoryTree>(Tree<Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>>>::getOrigin());
    }

    std::vector<std::shared_ptr<HistoryTree>> HistoryTree::getChildren() const
    {
        std::vector<std::shared_ptr<HistoryTree>> vector;
        for (const auto &children : Tree<Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>>>::getChildren())
        {
            vector.push_back(std::static_pointer_cast<HistoryTree>(children));
        }
        return vector;
    }

    std::shared_ptr<HistoryTree> HistoryTree::getChild(const Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>> &obs_act) const
    {
        return std::static_pointer_cast<HistoryTree>(Tree<Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>>>::getChild(obs_act));
    }

    template <class Archive>
    void HistoryTree::serialize(Archive &, const unsigned int)
    {
        // using boost::serialization::make_nvp;
        // archive &boost::serialization::base_object<Tree<Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>>>>(*this);
    }

    const Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>> &HistoryTree::getData() const
    {
        return Tree<Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>>>::getData();
    }

} // namespace sdm