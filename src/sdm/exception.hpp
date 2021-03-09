/**
 * @file exception.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief This file contains implemented exception that can be throw in your code.
 * @version 1.0
 * @date 29/01/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <string>
#include <exception>

namespace sdm
{
    /**
     * @brief Namespace for exceptions
     */
    namespace exception
    {

        /**
         * @brief Base exception class
         */
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

        class NotImplementedException : public Except
        {
        public:
            explicit NotImplementedException() : Except("Not Implemented Exception") {}
        };

        /**
         * @brief Developpers use this class to raise a file not found exception 
         */
        class FileNotFoundException : public Except
        {
        private:
            std::string file;

        public:
            /** Constructor (C++ STL string).
             *  @param file_ The file
             */
            explicit FileNotFoundException(std::string file_) : Except("File \"" + file_ + "\" does not seem to exists."),
                                                                file(file_) {}
            std::string get_file() const { return file; }
        };

        /**
         * @brief Developpers use this class to raise a parsing exception 
         */
        class ParsingException : public Except
        {
        protected:
            std::string line_details;

        public:
            /** Constructor (C++ STL string).
             *  @param line_details_ The line where error occures
             */
            explicit ParsingException(const std::string &line_details_ = "") : Except("Parsing failed -> Stopped at: \"" + line_details_ + "\"\n"),
                                                                               line_details(line_details_) {}
            std::string get_line_details() const { return line_details; }
        };

    } // namespace exception
} // namespace sdm