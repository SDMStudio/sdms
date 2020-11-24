#include <exception>
#include <string>

namespace sdm
{
    //! \file     exception.hpp
    //! \author   David Albert
    //! \brief    File containing several classes of exception
    //! \version  1.0
    //! \date     10 Novembre 2020

    namespace exception
    {

        //! \class  Except  exception.hpp
        //! \brief  Generic Exception class
        class Except : virtual public std::exception
        {

        protected:
            //! \brief Error message
            std::string error_message; 

        public:
            /** Constructor (C++ STL string).
             *  @param msg_ The error message
             */
            explicit Except(const std::string &msg_) : error_message(msg_)
            {
            }

            /** Returns a pointer to the (constant) error description.
             *  @return A pointer to a const char*. The underlying memory
             *  is in possession of the Except object. Callers must
             *  not attempt to free the memory.
             */
            virtual const char *what() const throw()
            {
                return error_message.c_str();
            }
        };

        //! \class  ParsingException  exception.hpp
        //! \brief  ParsingException class
        class ParsingException : public Except
        {
        protected:
            //! \brief The line where error occures
            std::string line_details;

        public:
            /** Constructor (C++ STL string).
             *  @param line_details_ The line where error occures
             */
            explicit ParsingException(const std::string &line_details_ = "") : Except("Parsing failed -> Stopped at: \"" + line_details_ + "\"\n"),
                                                                                                         line_details(line_details_)
            {
            }

            std::string get_line_details() const { return line_details; }
        };

        //! \class  FileNoteFoundException  exception.hpp
        //! \brief  FileNoteFoundException class
        class FileNotFoundException : public Except
        {
        private:
            //! \brief The file
            std::string file;

        public:
            /** Constructor (C++ STL string).
             *  @param file_ The file
             */
            explicit FileNotFoundException(std::string file_) : Except("File \"" + file_ + "\" does not seem to exists."),
                                                                file(file_)
            {
            }

            std::string get_file() const { return file; }
        };

    } // namespace exception
} // namespace sdm