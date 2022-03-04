#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/hyperplane/alpha_vector.hpp>

namespace sdm
{
    class Factored_oAlpha : public AlphaVector
    {
    public:
        /**
         * @brief Defines the group's history representation.
         */
        using HistoryGroup = std::vector<std::shared_ptr<HistoryInterface>>;

        /**
         * @brief Defines the group's state representation.
         */
        using StateGroup = std::vector<std::shared_ptr<State>>;

        /**
         * @brief Defines the group's hyperplane structure.
         */
        using AlphaGroup = std::unordered_map<HistoryGroup, MappedVector<StateGroup, double>>;

        Factored_oAlpha(double default_value);

        bool isDominated(const Hyperplane &other) const;

        double getValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o) const;
        void setValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &o, double value);
        double getBetaValueAt(const std::shared_ptr<State> &x, const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<Action> &u, const std::shared_ptr<POMDPInterface> &pomdp, number t);

        size_t hash(double precision) const;
        
        bool isEqual(const Factored_oAlpha &other, double precision) const;
        bool isEqual(const std::shared_ptr<Hyperplane> &other, double precision) const;

        size_t size() const;
        std::string str() const;

        AlphaGroup& getAlphaGroup(int group_id);

    protected:
        /**
         * @brief Keep the group's hyperlane set. 
         */
        std::vector<AlphaGroup> container;
    };
}
