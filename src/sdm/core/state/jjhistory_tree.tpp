
#include <sdm/core/state/jjhistory_tree.hpp>

namespace sdm
{

    template <typename T>
    JointJointHistoryTree<T>::JointJointHistoryTree() : JointHistoryTree<Joint<T>>()
    {
    }

    template <typename T>
    JointJointHistoryTree<T>::JointJointHistoryTree(number n_agents_minus_one) : JointHistoryTree<Joint<T>>()
    {
        for (number i = 0; i < n_agents_minus_one; i++)
        {
            this->indiv_jhists.push_back(std::make_shared<JointHistoryTree<T>>(n_agents_minus_one + 1 - i));
        }
    }

    template <typename T>
    JointJointHistoryTree<T>::JointJointHistoryTree(number n_agents_minus_one, number max_depth) : JointHistoryTree<Joint<T>>(max_depth)
    {
        for (number i = 0; i < n_agents_minus_one; i++)
        {
            this->indiv_jhists.push_back(std::make_shared<JointHistoryTree<T>>(n_agents_minus_one + 1 - i, max_depth));
        }
    }

    template <typename T>
    JointJointHistoryTree<T>::JointJointHistoryTree(std::shared_ptr<JointJointHistoryTree<T>> parent, 
                                          const Joint<T> &item) : JointHistoryTree<Joint<T>>(parent, item)
    {
    }

    template <typename T>
    JointJointHistoryTree<T>::JointJointHistoryTree(std::vector<jhistory_type> trees){
        for (number i = 0; i < trees.size(); i++)
        {
            this->indiv_jhists.push_back(trees[i]);
        }
    }

    template <typename T>
    void JointJointHistoryTree<T>::addIndivJHist(std::shared_ptr<JointHistoryTree<T>> jhist)
    {
        this->indiv_jhists.push_back(jhist);
    }

    template <typename T>
    std::shared_ptr<JointJointHistoryTree<T>> JointJointHistoryTree<T>::expand(const Joint<Joint<T>> &data, bool backup)
    {
        std::shared_ptr<JointJointHistoryTree<T>> h_joint_joint = HistoryTree<Joint<Joint<T>>>::template expand<JointJointHistoryTree<T>>(data, backup);

        if (h_joint_joint->getIndividualJointHistories().size() == 0)
        {
            for (number i = 0; i < this->indiv_jhists.getNumAgents(); i++)
            {
                h_joint_joint->addIndivJHist(this->getIndividualJointHistory(i)->expand(data[i], backup));
            }
        }
        return h_joint_joint;
    }

    template <typename T>
    std::shared_ptr<JointHistoryTree<T>> JointJointHistoryTree<T>::getIndividualJointHistory(number ag_id) const
    {
        assert((ag_id < this->indiv_jhists.getNumAgents()) && (ag_id >= 0));
        return this->indiv_jhists.at(ag_id);
    }

    template <typename T>
    std::vector<std::shared_ptr<JointHistoryTree<T>>> JointJointHistoryTree<T>::getIndividualJointHistories() const
    {
        // std::vector<std::shared_ptr<HistoryTree<T>>> ihists;
        // for (number i = 0; i < this->indiv_jhists.getNumAgents(); i++)
        // {
        //     ihists.push_back(this->getIndividualHistory(i));
        // }
        return this->indiv_jhists;
    }

} // namespace sdm