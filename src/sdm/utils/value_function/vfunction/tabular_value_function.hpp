
#pragma once

#include <sdm/utils/config.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/vfunction/tabular_vf_interface.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief Tabular value functions are state value functions represented by a vector. 
     * 
     * The size of the vector depends on the number of existing states. 
     * 
     * @tparam Hash the type of hash function
     * @tparam KeyEqual the type of equal function
     */
    template <class Hash = std::hash<std::shared_ptr<State>>, class KeyEqual = std::equal_to<std::shared_ptr<State>>>
    class BaseTabularValueFunction : public ValueFunction, public TabularValueFunctionInterface
    {
    public:
        using Container = MappedVector<std::shared_ptr<State>, double, Hash, KeyEqual>;

        BaseTabularValueFunction(const std::shared_ptr<SolvableByDP> &world = nullptr,
                                 const std::shared_ptr<Initializer> &initialize = nullptr,
                                 const std::shared_ptr<ActionSelectionInterface> &action_selection = nullptr,
                                 Config config = {});

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
         * @brief Set the value function at state s and timestep t to a new value.
         *
         * @param state the state
         * @param new_value the new value
         * @param t the timestep
         */
        void setValueAt(const std::shared_ptr<State> &state, double new_value, number t);

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
         * 
         * The extension of the file will indicate the type of formatage for reading
         * (`.txt` = text format, '.xml' = XML format, other = binary format).
         *
         * @param filename the filename
         */
        void load(std::string filename);

        /**
         * @brief Copy the value function and return a reference to the copied object.
         * 
         * @return the address of the value function copied
         */
        std::shared_ptr<ValueFunctionInterface> copy();

        /**
         * @brief Get a string representation of this class.
         */
        std::string str() const;

        /**
         * @brief Return the possible indexes of the value function
         */
        std::vector<std::shared_ptr<State>> getSupport(number t);

        /**
         * @brief Get the representation structure of the value function. 
         * 
         * @param t the time step
         * @return the container
         */
        Container &getRepresentation(number t);

        /**
         * @brief Get the size of the value function at timestep t
         */
        size_t getSize(number t) const;
        

        friend std::ostream &operator<<(std::ostream &os, BaseTabularValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

        // Will soon be deprecated
        void do_pruning(number t);

    protected:
        /**
         * @brief The value function represention.
         *
         * The default representation is a MappedVector but every class implementing VectorInterface interface can be used.
         */
        std::vector<Container> representation;

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

    /** @brief Tabular value function using address comparison */
    using TabularValueFunction = BaseTabularValueFunction<std::hash<std::shared_ptr<State>>, std::equal_to<std::shared_ptr<State>>>;
    
    /** @brief Point set value function using state content comparison */
    using TabularValueFunction2 = BaseTabularValueFunction<sdm::hash_from_ptr<State>, sdm::equal_from_ptr<State>>;

} // namespace sdm

#include <sdm/utils/value_function/vfunction/tabular_value_function.tpp>