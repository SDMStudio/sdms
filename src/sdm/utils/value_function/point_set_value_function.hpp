#pragma once

#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/backup/backup_interface.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    class PointSetValueFunction : public TabularValueFunction
    {
    public:
        using Container = MappedVector<std::shared_ptr<State>, double>;

        PointSetValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer, const std::shared_ptr<BackupInterface> &backup, int freq_prunning = 10);

        PointSetValueFunction(number horizon,double default_value = 0., const std::shared_ptr<BackupInterface> &backup =nullptr, int freq_prunning = 10);

        /**
         * @brief Update the value at a specific state and timestep.
         * 
         * @param state the state
         * @param t the timestep. Must be less than the horizon, $t < h$. Except in serialized problem solving where real timesteps are serialized and thus we need $t < h \times n$. 
         */
        void updateValueAt(const std::shared_ptr<State> &state, number t, double target);

        bool is_dominated(const std::shared_ptr<State> &ostate, double value, number t);

        std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, PointSetValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

    protected:
        /**
         * @brief Frequency before prunning.
         */
        number freq_prune_;

        /**
         * @brief The last time the prunning took place.
         * 
         */
        number last_prunning = 0;

        /**
         * @brief epsilon value used for the prunning
         * 
         */
        double epsilon_prunning;

        /**
         * @brief Point-wise pruning.
         * 
         */
        void prune(number t = 0);


    public:
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &archive, const unsigned int)
        {
            using boost::serialization::make_nvp;

            archive &make_nvp("horizon", this->horizon_);
            archive &make_nvp("representation", representation);
        }
    };

} // namespace sdm
