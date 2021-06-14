#pragma once

#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/value_function/backup/backup_base.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    class PointSetValueFunction : public ValueFunction
    {
    public:
        using Container = MappedVector<std::shared_ptr<State>, double>;

        PointSetValueFunction(std::shared_ptr<BackupInterface>,number horizon, std::shared_ptr<Initializer> initializer);

        PointSetValueFunction(std::shared_ptr<BackupInterface>, number horizon = 0, double default_value = 0.);

        /**
         * @brief Initialize the value function according using initializer.
         * 
         */
        void initialize();

        /**
         * @brief Set all values of the vector to a default value. 
         * 
         * @param default_value the default value 
         */
        void initialize(double default_value, number t = 0);

        /**
         * @brief Evaluate the value at a state.
         * 
         * @param state the state where we want to evaluate the function
         * @return the value
         */
        double getValueAt(const std::shared_ptr<State> &state, number t = 0);

        /**
         * @brief Update the value at a specific state and timestep.
         * 
         * @param state the state
         * @param t the timestep. Must be less than the horizon, $t < h$. Except in serialized problem solving where real timesteps are serialized and thus we need $t < h \times n$. 
         */
        void updateValueAt(const std::shared_ptr<State> &state, number t = 0);
        void updateValueAt(const std::shared_ptr<State> &state, number t, double target);

        // /**
        //  * @brief Save a value function into a file. 
        //  * The extension of the file will indicate the type of formatage for recording (`.txt` = text format, '.xml' = XML format, other = binary format). 
        //  * 
        //  * @param filename the filename
        //  */
        // void save(std::string filename);

        // /**
        //  * @brief Load a value function from a file.
        //  * The extension of the file will indicate the type of formatage for reading (`.txt` = text format, '.xml' = XML format, other = binary format). 
        //  * 
        //  * @param filename the filename
        //  */
        // void load(std::string filename);

        std::string str() const;

        std::vector<std::shared_ptr<State>> getSupport(number t);

        friend std::ostream &operator<<(std::ostream &os, PointSetValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

    protected:
        /**
         * @brief The temporary one-stage value function represention.
         */
        Container tmp_representation;

        /**
         * @brief The value function represention.
         * The default representation is a MappedVector but every class implementing VectorInterface interface can be used.
         */
        std::vector<Container> representation;


        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<Initializer> initializer_;

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

        bool is_dominated(const std::shared_ptr<State> &ostate, double value, number t);

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

    // using MappedValueFunction = PointSetValueFunction<MappedVector>;

    // using SparseValueFunction = PointSetValueFunction<SparseVector>;

    // using DenseValueFunction = PointSetValueFunction<DenseVector>;

} // namespace sdm
#include <sdm/utils/value_function/point_set_value_function.tpp>
