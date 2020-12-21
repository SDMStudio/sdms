#include <sdm/core/state/history_tree.hpp>

namespace sdm
{
    template <typename T>
    HistoryTree<T>::HistoryTree() : Tree<T>(), State
    {
    }

    template <typename T>
    HistoryTree<T>::HistoryTree(HistoryTree *parent, const T &item, bool backup) : Tree<T>(parent, item, backup)
    {
    }

    template <typename T>
    HistoryTree<T>::~HistoryTree()
    {
    }

    template <typename T>
    HistoryTree<T> *HistoryTree<T>::expand(const T &data)
    {
        std::cout << "expand" << "\n";
    }

} // namespace sdm