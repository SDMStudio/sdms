
#define DEFINE_JOINT(CLASS)                                                                                                                            \
    /* This macro allow to define a specific joint on std::shared_ptr<CLASS> that inherites from CLASS. */                                             \
    template <>                                                                                                                                        \
    class Joint<std::shared_ptr<CLASS>> : public CLASS, public std::vector<std::shared_ptr<CLASS>>, public Function<number, std::shared_ptr<CLASS>>    \
    {                                                                                                                                                  \
    public:                                                                                                                                            \
        using value_type = std::shared_ptr<CLASS>;                                                                                                     \
                                                                                                                                                       \
        Joint() {}                                                                                                                                     \
        Joint(std::size_t size) : std::vector<std::shared_ptr<CLASS>>(size) {}                                                                         \
        Joint(std::size_t size, std::shared_ptr<CLASS> default_value) : std::vector<std::shared_ptr<CLASS>>(size, default_value) {}                    \
        Joint(const std::vector<std::shared_ptr<CLASS>> &joint_item) : std::vector<std::shared_ptr<CLASS>>(joint_item) {}                              \
        Joint(const std::vector<number> &, const std::vector<std::shared_ptr<CLASS>> &joint_item) : std::vector<std::shared_ptr<CLASS>>(joint_item) {} \
        Joint(std::initializer_list<std::shared_ptr<CLASS>> list_values) : std::vector<std::shared_ptr<CLASS>>(list_values) {}                         \
                                                                                                                                                       \
        const std::shared_ptr<CLASS> &get(const number &index) const                                                                                   \
        {                                                                                                                                              \
            return this->at(index);                                                                                                                    \
        }                                                                                                                                              \
                                                                                                                                                       \
        number getNumAgents() const                                                                                                                    \
        {                                                                                                                                              \
            return this->size();                                                                                                                       \
        }                                                                                                                                              \
                                                                                                                                                       \
        template <typename TOutput>                                                                                                                    \
        std::shared_ptr<Joint<std::shared_ptr<TOutput>>> toJoint()                                                                                     \
        {                                                                                                                                              \
            auto joint_output = std::make_shared<Joint<std::shared_ptr<TOutput>>>();                                                                   \
                                                                                                                                                       \
            for (const auto &item : *this)                                                                                                             \
            {                                                                                                                                          \
                joint_output->push_back(std::static_pointer_cast<TOutput>(item));                                                                      \
            }                                                                                                                                          \
                                                                                                                                                       \
            return joint_output;                                                                                                                       \
        }                                                                                                                                              \
                                                                                                                                                       \
        std::shared_ptr<CLASS> operator()(const number &i)                                                                                             \
        {                                                                                                                                              \
            return (*this)[i];                                                                                                                         \
        }                                                                                                                                              \
                                                                                                                                                       \
        std::string str() const                                                                                                                        \
        {                                                                                                                                              \
            std::ostringstream res;                                                                                                                    \
            res << "[" << this->getNumAgents() << "](";                                                                                                \
            if (this->getNumAgents() > 0)                                                                                                              \
            {                                                                                                                                          \
                number ag;                                                                                                                             \
                for (ag = 0; ag < this->getNumAgents() - 1; ++ag)                                                                                      \
                {                                                                                                                                      \
                    res << this->get(ag)->str() << ", ";                                                                                               \
                }                                                                                                                                      \
                res << this->get(ag)->str();                                                                                                           \
            }                                                                                                                                          \
            res << ")";                                                                                                                                \
            return res.str();                                                                                                                          \
        }                                                                                                                                              \
        friend std::ostream &operator<<(std::ostream &os, const Joint<std::shared_ptr<CLASS>> &joint_action)                                           \
        {                                                                                                                                              \
            os << joint_action.str();                                                                                                                  \
            return os;                                                                                                                                 \
        }                                                                                                                                              \
    };

namespace sdm
{

    // Specialisation for the Joint Action
    DEFINE_JOINT(Item);

    // Specialisation for the Joint Action
    DEFINE_JOINT(Action);

    // Specialisation for the Joint State
    DEFINE_JOINT(State);

    // Specialisation for the Joint Observation
    DEFINE_JOINT(Observation);

    template <typename T>
    Joint<T>::Joint() {}

    template <typename T>
    Joint<T>::Joint(std::size_t size) : std::vector<T>(size) {}

    template <typename T>
    Joint<T>::Joint(std::size_t size, T default_value) : std::vector<T>(size, default_value) {}

    template <typename T>
    Joint<T>::Joint(const std::vector<T> &joint_item) : std::vector<T>(joint_item) {}

    template <typename T>
    Joint<T>::Joint(const std::vector<number> &, const std::vector<T> &joint_item) : std::vector<T>(joint_item) {}

    template <typename T>
    Joint<T>::Joint(std::initializer_list<T> list_values) : std::vector<T>(list_values) {}

    template <typename T>
    Joint<T>::~Joint() {}

    template <typename T>
    const T &Joint<T>::get(const number &index) const
    {
        return this->at(index);
    }

    template <typename T>
    number Joint<T>::getNumAgents() const
    {
        return this->size();
    }

    template <typename T>
    template <typename TOutput>
    std::shared_ptr<Joint<std::shared_ptr<TOutput>>> Joint<T>::toJoint()
    {
        auto joint_output = std::make_shared<Joint<std::shared_ptr<TOutput>>>();

        for (const auto &item : *this)
        {
            joint_output->push_back(std::static_pointer_cast<TOutput>(item));
        }

        return joint_output;
    }

    template <typename T>
    T Joint<T>::operator()(const number &i)
    {
        return (*this)[i];
    }

    template <typename T>
    std::string Joint<T>::str() const
    {
        std::ostringstream res;
        res << "[" << this->getNumAgents() << "](";
        if (this->getNumAgents() > 0)
        {
            number ag;
            for (ag = 0; ag < this->getNumAgents() - 1; ++ag)
            {
                res << this->get(ag) << ", ";
            }
            res << this->get(ag);
        }
        res << ")";
        return res.str();
    }

} // namespace sdm

namespace std
{
    template <typename T>
    struct hash<sdm::Joint<T>>
    {
        typedef sdm::Joint<T> argument_type;
        typedef std::size_t result_type;
        result_type operator()(argument_type const &in) const
        {
            return std::hash<std::vector<T>>()(in);
        }
    };
}