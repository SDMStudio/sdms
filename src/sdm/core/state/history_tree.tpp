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
    template <typename output>
    std::shared_ptr<output> HistoryTree<T>::expand(const T &data, bool backup)
    {
        if (backup && (this->children_.find(data) != this->children_.end()))
        {
            return std::static_pointer_cast<output>(this->getChild(data));
        }
        if (backup && (this->getDepth() >= this->getMaxDepth()))
        {
            return this->truncatedExpand<output>(data, backup);
            // return this->template base_truncated_expand<output>(item, backup);
        }
        if (backup)
        {
            // this->addChild(data);
            this->children_.emplace(data, std::make_shared<output>(std::static_pointer_cast<output>(this->shared_from_this()), data));
            return std::static_pointer_cast<output>(this->getChild(data));
        }
        return std::make_shared<output>(std::static_pointer_cast<output>(this->shared_from_this()), data);
    }

    template <typename T>
    template <typename output>
    std::shared_ptr<output> HistoryTree<T>::truncatedExpand(const T &item, bool backup)
    {
        std::list<T> items;

        //<! fill in the vector of observation to simulate
        items.push_front(item);
        auto parent = this->shared_from_this();
        while (items.size() < this->getMaxDepth())
        {
            items.push_front(parent->getData());
            parent = std::static_pointer_cast<output>(parent->getParent());
        }

        //<! iteratively expands the base_graph
        // auto trace = std::static_cast<HistoryTree<T> *>(this->origin_);
        auto trace = std::static_pointer_cast<HistoryTree<T>>(parent->getOrigin());

        for (auto it = items.begin(); it != items.end(); ++it)
        {
            trace = trace->template expand<output>(*it, backup);
            // trace = trace->template base_expand<output>(*it, backup);
        }

        // if (backup)
        //     this->children.emplace(item, trace); //<! TODO WARNING this line might cause troubles!!!

        return std::static_pointer_cast<output>(trace);
    }

    template <typename T>
    std::string HistoryTree<T>::str()
    {
        std::ostringstream res;
        std::list<T> items;
        std::shared_ptr<Tree<T>> chistory = this->shared_from_this(), origin = this->getOrigin();
        res << "<history id=\"" << chistory << "\"  horizon=\"" << this->getDepth() << "\" value=\"";
        while (chistory != origin)
        {
            items.push_front(chistory->getData());
            chistory = chistory->getParent();
        }
        for (auto item : items)
        {
            res << item << " ";
        }
        res << "\"/>" << std::endl;
        return res.str();
    }

} // namespace sdm