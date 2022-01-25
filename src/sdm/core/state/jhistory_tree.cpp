
#include <sdm/core/state/jhistory_tree.hpp>

namespace sdm
{

    JointHistoryTree::JointHistoryTree()
    {
    }

    JointHistoryTree::JointHistoryTree(number n_agents)
    {
        for (number i = 0; i < n_agents; i++)
        {
            this->addIndividualHistory(std::make_shared<HistoryTree>());
        }
        this->setupDefaultObs(n_agents, sdm::NO_OBSERVATION);
    }

    JointHistoryTree::JointHistoryTree(number n_agents, number max_depth) : HistoryTree(max_depth)
    {
        for (number i = 0; i < n_agents; i++)
        {
            this->addIndividualHistory(std::make_shared<HistoryTree>(max_depth));
        }
        this->setupDefaultObs(n_agents, sdm::NO_OBSERVATION);
    }

    JointHistoryTree::JointHistoryTree(std::shared_ptr<HistoryTree> parent, const Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>> &item) : HistoryTree(parent, item)
    {
        this->default_observation_ = std::static_pointer_cast<JointHistoryTree>(parent)->getDefaultObs();
    }

    JointHistoryTree::JointHistoryTree(const Joint<std::shared_ptr<HistoryInterface>> &ihistories) : HistoryTree(), Joint<std::shared_ptr<HistoryInterface>>(ihistories)
    {
        this->setupDefaultObs(ihistories.size(), sdm::NO_OBSERVATION);
    }

    void JointHistoryTree::addIndividualHistory(std::shared_ptr<HistoryInterface> ihist)
    {
        this->push_back(ihist);
    }

    std::shared_ptr<HistoryInterface> JointHistoryTree::expand(const std::shared_ptr<Observation> &joint_observation, const std::shared_ptr<Action> &joint_action, bool backup)
    {
        return this->expand(std::static_pointer_cast<JointObservation>(joint_observation), std::static_pointer_cast<JointAction>(joint_action), backup);
    }

    std::shared_ptr<HistoryTree> JointHistoryTree::expandHistoryTree(const std::shared_ptr<Observation> &joint_observation, const std::shared_ptr<Action> &joint_action, bool backup)
    {
        return this->expandJointHistoryTree(std::static_pointer_cast<JointObservation>(joint_observation), std::static_pointer_cast<JointAction>(joint_action), backup);
    }

    std::shared_ptr<JointHistoryInterface> JointHistoryTree::expand(const std::shared_ptr<JointObservation> &joint_observation, const std::shared_ptr<JointAction> &joint_action, bool backup)
    {
        return this->expandJointHistoryTree(joint_observation, joint_action, backup);
    }

    std::shared_ptr<JointHistoryTree> JointHistoryTree::expandJointHistoryTree(const std::shared_ptr<JointObservation> &joint_observation, const std::shared_ptr<JointAction> &joint_action, bool backup)
    {
        std::shared_ptr<JointHistoryTree> h_joint;

        if (*joint_observation != *this->default_observation_)
        {
            h_joint = HistoryTree::template expand<JointHistoryTree>(joint_observation, joint_action, backup);
            if (h_joint->size() == 0)
            {
                for (number i = 0; i < this->getNumAgents(); i++)
                {
                    if (joint_action != nullptr)
                        h_joint->addIndividualHistory(this->getIndividualHistory(i)->expand(joint_observation->get(i), joint_action->get(i), backup));
                    else
                        h_joint->addIndividualHistory(this->getIndividualHistory(i)->expand(joint_observation->get(i), joint_action, backup));
                }
            }
        }
        else
        {
            h_joint = this->getptr();
        }

        return h_joint;
    }

    std::shared_ptr<HistoryInterface> JointHistoryTree::getIndividualHistory(number ag_id) const
    {
        assert((ag_id < this->getNumAgents()) && (ag_id >= 0));
        return this->get(ag_id);
    }

    Joint<std::shared_ptr<HistoryInterface>> JointHistoryTree::getIndividualHistories() const
    {
        return *this;
    }

    std::string JointHistoryTree::str() const
    {
        std::ostringstream res;
        res << "<joint-history id=\"" << this << "\"  horizon=\"" << this->getDepth() << "\" value=\"" << this->short_str() << "\"/>";
        return res.str();
    }

    void JointHistoryTree::setDefaultObs(const std::shared_ptr<JointObservation> &default_observation)
    {
        this->default_observation_ = default_observation;
    }

    void JointHistoryTree::setupDefaultObs(number n_agents, const std::shared_ptr<Observation> &default_observation)
    {
        auto default_joint_observation = std::make_shared<JointObservation>();
        for (number i = 0; i < n_agents; i++)
        {
            default_joint_observation->push_back(default_observation);
        }
        this->setDefaultObs(default_joint_observation);
    }

    std::shared_ptr<JointObservation> JointHistoryTree::getDefaultObs()
    {
        return this->default_observation_;
    }

    std::shared_ptr<JointHistoryTree> JointHistoryTree::getptr()
    {
        return std::static_pointer_cast<JointHistoryTree>(HistoryTree::getptr());
        // return Tree<Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>>>::downcasted_shared_from_this<JointHistoryTree>();
    }

    std::shared_ptr<JointHistoryTree> JointHistoryTree::getParent() const
    {
        return std::static_pointer_cast<JointHistoryTree>(HistoryTree::getParent());
    }

    std::shared_ptr<JointHistoryTree> JointHistoryTree::getOrigin()
    {
        return std::static_pointer_cast<JointHistoryTree>(HistoryTree::getOrigin());
    }

    std::vector<std::shared_ptr<JointHistoryTree>> JointHistoryTree::getChildren() const
    {
        std::vector<std::shared_ptr<JointHistoryTree>> vector;
        for (const auto &children : HistoryTree::getChildren())
        {
            vector.push_back(std::static_pointer_cast<JointHistoryTree>(children));
        }
        return vector;
    }

    std::shared_ptr<JointHistoryTree> JointHistoryTree::getChild(const Pair<std::shared_ptr<Observation>, std::shared_ptr<Action>> &child_item) const
    {
        return std::static_pointer_cast<JointHistoryTree>(HistoryTree::getChild(child_item));
    }

    template <class Archive>
    void JointHistoryTree::serialize(Archive &archive, const unsigned int)
    {
        using boost::serialization::make_nvp;

        archive &boost::serialization::base_object<HistoryTree>(*this);
        archive &boost::serialization::base_object<Joint<std::shared_ptr<HistoryTree>>>(*this);
        archive &make_nvp("default_observation", default_observation_);
    }

} // namespace sdm