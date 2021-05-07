
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

        this->setDefaultObs(n_agents);
    }

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree(number n_agents, number max_depth) : HistoryTree<Joint<T>>(max_depth)
    {
        for (number i = 0; i < n_agents; i++)
        {
            this->addIndividualHistory(std::make_shared<HistoryTree<T>>(max_depth));
        }
        this->setDefaultObs(n_agents);
    }

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree(std::shared_ptr<JointHistoryTree<T>> parent, const Joint<T> &item) : HistoryTree<Joint<T>>(parent, item)
    {
        this->setDefaultObs(item.size());
    }

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree(const Joint<std::shared_ptr<HistoryTree<T>>> &ihistories) : HistoryTree<Joint<T>>(), Joint<std::shared_ptr<HistoryTree<T>>>(ihistories)
    {
        this->setDefaultObs(ihistories.size());
    }

    template <typename T>
    void JointHistoryTree<T>::addIndividualHistory(std::shared_ptr<HistoryTree<T>> ihist)
    {
        this->push_back(ihist);
    }

    template <typename T>
    std::shared_ptr<JointHistoryTree<T>> JointHistoryTree<T>::expand(const Joint<T> &data, bool backup)
    {
        std::shared_ptr<JointHistoryTree<T>> h_joint;

        if (data != this->default_value)
        {
            h_joint = HistoryTree<Joint<T>>::template expand<JointHistoryTree<T>>(data, backup);
            if (h_joint->getIndividualHistories().size() == 0)
            {
                for (number i = 0; i < this->getNumAgents(); i++)
                {
                    h_joint->addIndividualHistory(this->getIndividualHistory(i)->expand(data[i], backup));
                }
            }
        }
        else
        {
            h_joint = this->getptr();
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

    template <typename T>
    void JointHistoryTree<T>::setDefaultObs(number n_agents)
    {
        std::vector<T> default_vector;
        
        for (number i = 0; i < n_agents; i++)
        {
            default_vector.push_back(sdm::DEFAULT_OBSERVATION);
        }

        this->default_value = Joint<T>(default_vector);
    }

    template <typename T>
    std::shared_ptr<JointHistoryTree<T>> JointHistoryTree<T>::getptr()
    {
        return std::static_pointer_cast<JointHistoryTree<T>>(this->shared_from_this());
    }

    template <typename T>
    std::shared_ptr<JointHistoryTree<T>> JointHistoryTree<T>::getParent() const
    {
        return std::static_pointer_cast<JointHistoryTree<T>>(HistoryTree<Joint<T>>::getParent());
    }

    template <typename T>
    std::shared_ptr<JointHistoryTree<T>> JointHistoryTree<T>::getOrigin()
    {
        return std::static_pointer_cast<JointHistoryTree<T>>(HistoryTree<Joint<T>>::getOrigin());
    }

    template <typename T>
    std::vector<std::shared_ptr<JointHistoryTree<T>>> JointHistoryTree<T>::getChildren() const
    {
        std::vector<std::shared_ptr<JointHistoryTree<T>>> vector;
        for(const auto& children : HistoryTree<Joint<T>>::getChildren())
        {
            vector.push_back(std::static_pointer_cast<JointHistoryTree<T>>(children));
        }
        return vector;
    }

    template <typename T>
    std::shared_ptr<JointHistoryTree<T>> JointHistoryTree<T>::getChild(const T &child_item) const
    {
        return std::static_pointer_cast<JointHistoryTree<T>>(HistoryTree<Joint<T>>::getChild(child_item));
    }

} // namespace sdm