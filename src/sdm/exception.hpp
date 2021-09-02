/**
 * @file exception.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief This file contains implementation for exceptions that can be thrown in your code.
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
     * @brief Namespace grouping all exceptions.
     */
    namespace exception
    {
        /**
         * @brief This class is the base class for SDMS exceptions. 
         * 
         * SDMS contains some exceptions for its internal usage. To define a new exception in SDMS, you need to add a class that inherite from the class sdm::exception::Exception.
         * 
         */
        class Exception : virtual public std::exception
        {
        public:
            /** 
             * @brief Constructor (C++ STL string).
             * @param msg_ The error message
             */
            explicit Exception(const std::string &msg_);

            /** 
             * @brief Returns a pointer to the (constant) error description.
             * 
             * @return A pointer to a const char*. The underlying memory
             * is in possession of the Except object. Callers must
             * not attempt to free the memory.
             */
            virtual const char *what() const throw();

        protected:
            //! \brief Error message
            std::string error_message;
        };

        /**
         * @brief Not implemented method exception.
         * 
         */
        class NotImplementedException : public Exception
        {
        public:
            explicit NotImplementedException();
        };

        /**
         * @brief File not found exception.
         */
        class FileNotFoundException : public Exception
        {
        public:
            /** Constructor (C++ STL string).
             *  @param file_ The file
             */
            explicit FileNotFoundException(std::string file_);

            /**
             * @brief Get the name of the file not found  
             * 
             * @return the filename
             */
            std::string get_file() const;
        private:
            std::string file;
        };

        /**
         * @brief Developpers use this class to raise a parsing exception.
         */
        class ParsingException : public Exception
        {
        public:
            /** Constructor (C++ STL string).
             * 
             *  @param line_details_ The line where error occurs.
             * 
             */
            explicit ParsingException(const std::string &line_details_ = "");

            /**
             * @brief Get details about lines that cause the failure.
             * 
             * @return line details
             */
            std::string get_line_details() const;

        protected:
            std::string line_details;
        };

    } // namespace exception
} // namespace sdm