

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
    }

#define SDMS_DECLARE_PARSER(CLASSNAME, PARSER_NAME) \
    std::shared_ptr<CLASSNAME> PARSER_NAME(std::string filename);

#define SDMS_INSTANCIATE_PARSER(GRAMMAR, TYPE_AST, TYPE_ENCODER, CLASSNAME, PARSER_NAME) \
                                                                                         \
    std::shared_ptr<CLASSNAME> PARSER_NAME(std::string filename)                         \
    {                                                                                    \
        TYPE_AST ast; /* Our tree */                                                     \
                                                                                         \
        std::ifstream in(filename, std::ios_base::in);                                   \
                                                                                         \
        if (!in)                                                                         \
        {                                                                                \
            throw sdm::exception::FileNotFoundException(std::string(filename));          \
        }                                                                                \
                                                                                         \
        std::string storage;         /* We will read the contents here. */               \
        in.unsetf(std::ios::skipws); /* No white space skipping! */                      \
        std::copy(                                                                       \
            std::istream_iterator<char>(in),                                             \
            std::istream_iterator<char>(),                                               \
            std::back_inserter(storage));                                                \
                                                                                         \
        /*  Defines spaces and comments */                                               \
        using boost::spirit::x3::eol;                                                    \
        using boost::spirit::x3::lexeme;                                                 \
        using boost::spirit::x3::ascii::char_;                                           \
        using boost::spirit::x3::ascii::space;                                           \
        auto const space_comment = space | lexeme['#' >> *(char_ - eol) >> eol];         \
                                                                                         \
        /*  Parse phrase (result in ast struct) */                                       \
        std::string::iterator begin = storage.begin();                                   \
        std::string::iterator iter = begin;                                              \
        std::string::iterator end = storage.end();                                       \
        bool r = phrase_parse(iter, end, GRAMMAR, space_comment, ast);                   \
        std::string context(iter, end);                                                  \
                                                                                         \
        if (r && iter == end)                                                            \
        {                                                                                \
            /* Convert ast to POSG class */                                              \
            TYPE_ENCODER encoder;                                                        \
            return encoder(ast);                                                         \
        }                                                                                \
        else                                                                             \
        {                                                                                \
            std::string::iterator deb_line = iter, end_line = iter;                      \
            while (deb_line != begin && *(deb_line - 1) != '\n')                         \
                deb_line = deb_line - 1;                                                 \
            while (end_line != end && *end_line != '\n')                                 \
                end_line = end_line + 1;                                                 \
                                                                                         \
            std::string context(deb_line, end_line);                                     \
            throw sdm::exception::ParsingException(context);                             \
        }                                                                                \
    }
