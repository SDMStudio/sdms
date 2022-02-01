

#define SDMS_REGISTRY(name_namespace) \
    namespace sdm                     \
    {                                 \
        namespace name_namespace      \
        {                             \
            registry::map_type registry::container = {

#define SDMS_END_REGISTRY() \
    }                       \
    ;                       \
    }                       \
    }

#define SDMS_SUB_REGISTRY(name_namespace1, name_namespace2) \
    namespace sdm                                           \
    {                                                       \
        namespace name_namespace1                           \
        {                                                   \
            namespace name_namespace2                       \
            {                                               \
                registry::map_type registry::container = {

#define SDMS_END_SUB_REGISTRY() \
    }                           \
    ;                           \
    }                           \
    }                           \
    }

#define SDMS_SUB_SUB_REGISTRY(name_namespace1, name_namespace2, name_namespace3) \
    namespace sdm                                                                \
    {                                                                            \
        namespace name_namespace1                                                \
        {                                                                        \
            namespace name_namespace2                                            \
            {                                                                    \
                namespace name_namespace3                                        \
                {                                                                \
                    registry::map_type registry::container = {

#define SDMS_END_SUB_SUB_REGISTRY() \
    }                               \
    ;                               \
    }                               \
    }                               \
    }                               \
    }

#define SDMS_REGISTER(name, class) {name, &createInstance<class>},

#define DEFINE_STD_HASH(CLASS_NAME, DEFAULT_PRECISION)                                  \
    namespace std                                                                       \
    {                                                                                   \
        template <>                                                                     \
        struct hash<CLASS_NAME>                                                         \
        {                                                                               \
            inline std::size_t operator()(const CLASS_NAME &in, double precision) const \
            {                                                                           \
                return in.hash(precision);                                              \
            }                                                                           \
                                                                                        \
            inline std::size_t operator()(const CLASS_NAME &in) const                   \
            {                                                                           \
                return in.hash(DEFAULT_PRECISION);                                      \
            }                                                                           \
        };                                                                              \
    }                                                                                   \