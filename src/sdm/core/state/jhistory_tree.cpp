
#include <sdm/core/state/jhistory_tree.hpp>

namespace sdm
{

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree()
    {
    }

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree(std::shared_ptr<JointHistoryTree> parent, const T &item, bool is_marked = true)
    {
    }

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree(const std::vector<T> &, const std::vector<action> &)
    {
    }

    template <typename T>
    JointHistoryTree<T>::~JointHistoryTree()
    {
    }
    
    template <typename T>
    void JointHistoryTree<T>::addChild(const std::vector<T> &child_items)
    {
        this->children_.emplace(child_item, new JointHistoryTree<T>(this, child_item, false));

    }

    template <typename T>
    void JointHistoryTree<T>::addChildren(const std::vector<T> &child_items)
    {
        for (int i = 0; i < child_items.size(); i++)
        {
            this->children_.emplace(child_items[i], new Tree<T>(this, child_items[i], false));
        }
    }
    
    template <typename T>
    HistoryTree<T> JointHistoryTree<T>::getHistory(number ag_id)
    {
        return this[ag_id];
    }
} // namespace sdm