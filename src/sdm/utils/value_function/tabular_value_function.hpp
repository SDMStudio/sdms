#pragma once

#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/backup/backup_interface.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    template <class Hash = std::hash<std::shared_ptr<State>>, class KeyEqual = std::equal_to<std::shared_ptr<State>>>
    class BaseTabularValueFunction : public ValueFunction
    {
    public:
        using Container = MappedVector<std::shared_ptr<State>, double, Hash, KeyEqual>;

        BaseTabularValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer, const std::shared_ptr<BackupInterfaceForValueFunction> &backup, const std::shared_ptr<ActionVFInterface> &action_vf);
        BaseTabularValueFunction(number horizon = 0, double default_value = 0., const std::shared_ptr<BackupInterfaceForValueFunction> &backup = nullptr, const std::shared_ptr<ActionVFInterface> &action_vf = nullptr);

        /**
         * @brief Initialize the value function by using initializer.
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

        virtual Pair<std::shared_ptr<State>, double> evaluate(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Update the value at a specific state and timestep.
         * 
         * @param state the state
         * @param t the timestep. Must be less than the horizon, $t < h$. Except in serialized problem solving where real timesteps are serialized and thus we need $t < h \times n$. 
         */
        void updateValueAt(const std::shared_ptr<State> &state, number t = 0);

        /**
         * @brief Update the value function at state s and timestep t to the target value.
         * 
         * @param state the state
         * @param t the timestep
         * @param target the target
         */
        virtual void updateValueAt(const std::shared_ptr<State> &state, number t, double target);

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

        size_t getSize(number t) const;

        friend std::ostream &operator<<(std::ostream &os, BaseTabularValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

        Container getRepresentation(number t);

    protected:
        /**
         * @brief The value function represention.
         * The default representation is a MappedVector but every class implementing VectorInterface interface can be used.
         */
        std::vector<Container> representation;

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

    using TabularValueFunction = BaseTabularValueFunction<std::hash<std::shared_ptr<State>>, std::equal_to<std::shared_ptr<State>>>;
    using TabularValueFunction2 = BaseTabularValueFunction<sdm::hash_from_ptr<State>, sdm::equal_from_ptr<State>>;

    // using SparseValueFunction = BaseTabularValueFunction<SparseVector>;

    // using DenseValueFunction = BaseTabularValueFunction<DenseVector>;

} // namespace sdm

#include <sdm/utils/value_function/tabular_value_function.tpp>