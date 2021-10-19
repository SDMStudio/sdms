#pragma once

#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/vfunction/tabular_vf_interface.hpp>
#include <sdm/utils/value_function/update_operator/vupdate_operator.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    template <class Hash = std::hash<std::shared_ptr<State>>, class KeyEqual = std::equal_to<std::shared_ptr<State>>>
    class BaseTabularValueFunction : public TabularValueFunctionInterface
    {
    public:
        using Container = MappedVector<std::shared_ptr<State>, double, Hash, KeyEqual>;

        BaseTabularValueFunction(number horizon, const std::shared_ptr<Initializer> &initialize = nullptr,
                                 const std::shared_ptr<ActionSelectionInterface> &action_selectionr = nullptr,
                                 const std::shared_ptr<TabularUpdateOperator> &update_operator = nullptr,
                                 bool is_upper_bound = false);

        BaseTabularValueFunction(const BaseTabularValueFunction &copy);

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

        /**
         * @brief Evaluate operator.
         *
         * In the tabular case, the evaluation only consists of getting the value in the table.
         * However, with other structures it can consists of a more complex operator such as making an interpolation.
         *
         * @param state the state
         * @param t the time step
         * @return a pair (state, value) of the value at this state
         */
        virtual Pair<std::shared_ptr<State>, double> evaluate(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Set the value function at state s and timestep t to a new value.
         *
         * @param state the state
         * @param new_value the new value
         * @param t the timestep
         */
        void setValueAt(const std::shared_ptr<State> &state, double new_value, number t);

        /**
         * @brief Save a value function into a file.
         *
         * The extension of the file will indicate the type of formatage for recording
         * (`.txt` = text format, '.xml' = XML format, other = binary format).
         *
         * @param filename the filename
         */
        void save(std::string filename);

        /**
         * @brief Load a value function from a file.
         * The extension of the file will indicate the type of formatage for reading
         * (`.txt` = text format, '.xml' = XML format, other = binary format).
         *
         * @param filename the filename
         */
        void load(std::string filename);

        std::string str() const;

        /**
         * @brief Return the possible indexes of the value function
         *
         * @return std::string
         */
        std::vector<std::shared_ptr<State>> getSupport(number t);

        size_t getSize(number t) const;

        friend std::ostream &operator<<(std::ostream &os, BaseTabularValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

        Container getRepresentation(number t);

        void do_pruning(number t);

        double getDefaultAt(number t);

    protected:
        /**
         * @brief The value function represention.
         *
         * The default representation is a MappedVector but every class implementing VectorInterface interface can be used.
         */
        std::vector<Container> representation;

        bool is_upper_bound_;

    public:
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &archive, const unsigned int &)
        {

            using boost::serialization::make_nvp;

            archive &make_nvp("horizon", this->horizon_);
            archive &make_nvp("representation", representation);
        }
    };

    using TabularValueFunction = BaseTabularValueFunction<std::hash<std::shared_ptr<State>>, std::equal_to<std::shared_ptr<State>>>;
    using TabularValueFunction2 = BaseTabularValueFunction<sdm::hash_from_ptr<State>, sdm::equal_from_ptr<State>>;

} // namespace sdm

#include <sdm/utils/value_function/vfunction/tabular_value_function.tpp>