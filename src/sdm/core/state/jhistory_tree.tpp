
#include <sdm/core/state/jhistory_tree.hpp>

namespace sdm
{

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree() : HistoryTree<Joint<T>>()
    {
    }

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree(number n_agents) : HistoryTree<Joint<T>>()
    {
        for (number i = 0; i < n_agents; i++)
        {
            this->indiv_hist.push_back(std::make_shared<HistoryTree<T>>());
        }

        this->setDefaultObs(n_agents);
    }

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree(number n_agents, number max_depth) : HistoryTree<Joint<T>>(max_depth)
    {
        for (number i = 0; i < n_agents; i++)
        {
            this->indiv_hist.push_back(std::make_shared<HistoryTree<T>>(max_depth));
        }
        this->setDefaultObs(n_agents);
    }

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree(std::shared_ptr<JointHistoryTree<T>> parent, const Joint<T> &item) : HistoryTree<Joint<T>>(parent, item)
    {
    }

    template <typename T>
    JointHistoryTree<T>::JointHistoryTree(const Joint<std::shared_ptr<HistoryTree<T>>> &ihistories) : HistoryTree<Joint<T>>(), indiv_hist(ihistories)
    {
        
    }

    template <typename T>
    void JointHistoryTree<T>::addIndivHist(std::shared_ptr<HistoryTree<T>> ihist)
    {
        this->indiv_hist.push_back(ihist);
    }

    template <typename T>
    std::shared_ptr<JointHistoryTree<T>> JointHistoryTree<T>::expand(const Joint<T> &data, bool backup)
    {
        //std::cout<<"\n default value "<<this->default_value;

        std::shared_ptr<JointHistoryTree<T>> h_joint = HistoryTree<Joint<T>>::template expand<JointHistoryTree<T>>(data, backup);
        if( data != this->default_value)
        {
            if (h_joint->getIndividualHistories().size() == 0)
            {
                for (number i = 0; i < this->indiv_hist.getNumAgents(); i++)
                {
                    h_joint->addIndivHist(this->getIndividualHistory(i)->expand(data[i], backup));
                }
            }
        }

        return h_joint;
    }

    template <typename T>
    std::shared_ptr<HistoryTree<T>> JointHistoryTree<T>::getIndividualHistory(number ag_id) const
    {
        assert((ag_id < this->indiv_hist.getNumAgents()) && (ag_id >= 0));
        return this->indiv_hist.at(ag_id);
    }

    template <typename T>
    std::vector<std::shared_ptr<HistoryTree<T>>> JointHistoryTree<T>::getIndividualHistories() const
    {
        return this->indiv_hist;
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

} // namespace sdm