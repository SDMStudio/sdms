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
     * @tparam std::shared_ptr<Item> Type of the states 
     * @tparam std::shared_ptr<Action> Type of the states
     * @tparam double Type of the values (must be primitive type)
     * @tparam TStruct Type of vector container (MappedVector, DenseVector and SparseVector are common type) 
     */
    template <class TItem,
              class TBackupOperator = ClassicBellmanBackupOperator,
              template <typename TI, typename TV> class TStruct = MappedVector>
    class TabularValueFunction : public ValueFunction,
                                 public BoostSerializable<TabularValueFunction<TItem, TBackupOperator, TStruct>>
    {
    public:
        using Container = TStruct<TItem, double>;
        using backup_operator_type = TBackupOperator;

        TabularValueFunction(std::shared_ptr<SolvableByHSVI> problem, number horizon, std::shared_ptr<Initializer> initializer);

        TabularValueFunction(std::shared_ptr<SolvableByHSVI> problem = nullptr, number horizon = 0, double default_value = 0.);

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
        double getValueAt(const std::shared_ptr<Item> &state, number t = 0);

        std::shared_ptr<Action> getBestAction(const std::shared_ptr<Item> &state, number t = 0);

        /**
         * @brief Update the value at a specific state and timestep.
         * 
         * @param state the state
         * @param t the timestep. Must be less than the horizon, $t < h$. Except in serialized problem solving where real timesteps are serialized and thus we need $t < h \times n$. 
         */
        void updateValueAt(const std::shared_ptr<Item> &state, number t = 0);
        void updateValueAt(const std::shared_ptr<Item> &state, number t, double target);

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

        virtual std::string str() const;

        std::vector<std::shared_ptr<Item>> getSupport(number t);

        backup_operator_type getBackupOperator();

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
         * @brief The backup operator used in order to update this value function. 
         * 
         */
        backup_operator_type backup_op_;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<Initializer> initializer_;

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

    template <typename TItem>
    using MappedValueFunction = TabularValueFunction<TItem, ClassicBellmanBackupOperator, MappedVector>;

    template <typename TItem>
    using SparseValueFunction = TabularValueFunction<TItem, ClassicBellmanBackupOperator, SparseVector>;

    template <typename TItem>
    using DenseValueFunction = TabularValueFunction<TItem, ClassicBellmanBackupOperator, DenseVector>;

} // namespace sdm
#include <sdm/utils/value_function/tabular_value_function.tpp>
