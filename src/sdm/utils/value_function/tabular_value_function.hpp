/**
 * @file tabular_value_function.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief Tabular value function are functions of state and action that use a vector representation. 
 * @version 0.1
 * @date 16/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <map>
#include <iostream>
#include <type_traits>

#include <sdm/types.hpp>
#include <sdm/tools.hpp>
#include <sdm/core/function.hpp>
#include <sdm/utils/struct/vector.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/sdms_vector.hpp>
#include <sdm/utils/backup_operator/backup_operator.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/initializer.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief Tabular value function are functions of state and action that use a vector representation to store the values. 
     * 
     * @tparam TState Type of the states 
     * @tparam TAction Type of the states
     * @tparam TValue Type of the values (must be primitive type)
     * @tparam TStruct Type of vector container (MappedVector, DenseVector and SparseVector are common type) 
     */
    template <typename TState,
              typename TAction,
              typename TValue = double,
              template <typename TI, typename TV> class TBackupOperator = ClassicBellmanBackupOperator,
              template <typename TI, typename TV> class TStruct = MappedVector>
    class TabularValueFunction : public ValueFunction<TState, TAction, TValue>,
                                 public BoostSerializable<TabularValueFunction<TState, TAction, TValue, TBackupOperator, TStruct>>
    {
    public:
        using Container = TStruct<TState, TValue>;
        using backup_operator_type = TBackupOperator<TState, TAction>;

        TabularValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, std::shared_ptr<Initializer<TState, TAction>> initializer);

        TabularValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem = nullptr, number horizon = 0, TValue default_value = 0.);

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
        void initialize(TValue default_value, number t = 0);

        /**
         * @brief Evaluate the value at a state.
         * 
         * @param state the state where we want to evaluate the function
         * @return the value
         */
        TValue getValueAt(const TState &state, number t = 0);

        TAction getBestAction(const TState &state, number t = 0);

        /**
         * @brief Update the value at a specific state and timestep.
         * 
         * @param state the state
         * @param t the timestep. Must be less than the horizon, $t < h$. Except in serialized problem solving where real timesteps are serialized and thus we need $t < h \times n$. 
         */
        void updateValueAt(const TState &state, number t = 0);
        void updateValueAt(const TState &state, number t, TValue target);

        /**
         * @brief Save a value function into a file. 
         * The extension of the file will indicate the type of formatage for recording (`.txt` = text format, '.xml' = XML format, other = binary format). 
         * 
         * @param filename the filename
         */
        void save(std::string filename);

        /**
         * @brief Load a value function from a file.
         * The extension of the file will indicate the type of formatage for reading (`.txt` = text format, '.xml' = XML format, other = binary format). 
         * 
         * @param filename the filename
         */
        void load(std::string filename);

        std::string str();

        std::vector<TState> getSupport(number t);

        backup_operator_type getBackupOperator();

        size_t getSize(number t) const;


        friend std::ostream &operator<<(std::ostream &os, TabularValueFunction<TState, TAction> &vf)
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
         * The default representation is a MappedVector but every class implementing VectorImpl interface can be used.
         */
        std::vector<Container> representation;

        /**
         * @brief The backup operator used in order to update this value function. 
         * 
         */
        backup_operator_type backup_op_;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<Initializer<TState, TAction>> initializer_;

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

    template <typename TState, typename TAction, typename TValue = double>
    using MappedValueFunction = TabularValueFunction<TState, TAction, TValue, ClassicBellmanBackupOperator, MappedVector>;

    template <typename TState, typename TAction, typename TValue = double>
    using SparseValueFunction = TabularValueFunction<TState, TAction, TValue, ClassicBellmanBackupOperator, SparseVector>;

    template <typename TState, typename TAction, typename TValue = double>
    using DenseValueFunction = TabularValueFunction<TState, TAction, TValue, ClassicBellmanBackupOperator, DenseVector>;

} // namespace sdm
#include <sdm/utils/value_function/tabular_value_function.tpp>
