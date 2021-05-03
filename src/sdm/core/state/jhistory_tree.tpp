
#include <sdm/core/state/jhistory_tree.hpp>

namespace sdm
{

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree()
    {
    }

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree(number n_agents)
    {
        for (number i = 0; i < n_agents; i++)
        {
            this->addIndividualHistory(std::make_shared<HistoryTree<T>>());
        }
    }

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree(number n_agents, number max_depth) : HistoryTree<Joint<T>>(max_depth)
    {
        for (number i = 0; i < n_agents; i++)
        {
            this->addIndividualHistory(std::make_shared<HistoryTree<T>>(max_depth));
        }
    }

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree(std::shared_ptr<JointHistoryTree<T>> parent, const Joint<T> &item) : HistoryTree<Joint<T>>(parent, item)
    {
    }

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree(const Joint<std::shared_ptr<HistoryTree<T>>> &ihistories) : HistoryTree<Joint<T>>(), Joint<std::shared_ptr<HistoryTree<T>>>(ihistories)
    {
    }

    template <typename T>
    void JointHistoryTree<T>::addIndividualHistory(std::shared_ptr<HistoryTree<T>> ihist)
    {
        this->push_back(ihist);
    }

    template <typename T>
    std::shared_ptr<JointHistoryTree<T>> JointHistoryTree<T>::expand(const Joint<T> &data, bool backup)
    {
        std::shared_ptr<JointHistoryTree<T>> h_joint = HistoryTree<Joint<T>>::template expand<JointHistoryTree<T>>(data, backup);

        if (h_joint->getNumAgents() == 0)
        {
            for (number i = 0; i < this->getNumAgents(); i++)
            {
                h_joint->addIndividualHistory(this->getIndividualHistory(i)->expand(data[i], backup));
            }
        }
        return h_joint;
    }

    template <typename T>
    std::shared_ptr<HistoryTree<T>> JointHistoryTree<T>::getIndividualHistory(number ag_id) const
    {
        assert((ag_id < this->getNumAgents()) && (ag_id >= 0));
        return this->get(ag_id);
    }

    template <typename T>
    Joint<std::shared_ptr<HistoryTree<T>>> JointHistoryTree<T>::getIndividualHistories() const
    {
        return *this;
    }

    template <typename T>
    std::string JointHistoryTree<T>::str()
    {
        return HistoryTree<Joint<T>>::str();
    }

} // namespace sdm