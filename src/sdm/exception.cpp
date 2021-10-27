#include <sdm/exception.hpp>

namespace sdm
{
    namespace exception
    {
        Exception::Exception(const std::string &msg_) : error_message(msg_)
        {
        }

        const char *Exception::what() const throw()
        {
            return error_message.c_str();
        }

        NotImplementedException::NotImplementedException() : Exception("Not Implemented Exception")
        {
        }

        FileNotFoundException::FileNotFoundException(std::string file_) : Exception("File \"" + file_ + "\" does not seem to exists."),
                                                                          file(file_)
        {
        }


        TypeError::TypeError(const std::string& msg_) : Exception(msg_)
        {
        }

        std::string FileNotFoundException::get_file() const
        {
            return file;
        }

        ParsingException::ParsingException(const std::string &line_details_) : Exception("Parsing failed -> Stopped at: \"" + line_details_ + "\"\n"),
                                                                                    line_details(line_details_)
        {
        }

        std::string ParsingException::get_line_details() const
        {
            return line_details;
        }

    } // namespace exception
} // namespace sdm