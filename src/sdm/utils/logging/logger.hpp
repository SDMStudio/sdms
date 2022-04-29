#pragma once

#include <iostream>
#include <locale>

#include <sdm/tools.hpp>
#ifdef __APPLE__
    #define FMT_HEADER_ONLY
#endif
#include <fmt/format.h>

namespace sdm
{
    /**
     * @brief This class provide a common interface for all loggers.
     * 
     */
    class BaseLogger
    {
    public:
        /**
         * @brief Record values.
         * 
         * @tparam TData... the types of input values
         * @param vals the values to log
         */
        template <class... TData>
        void log(TData... vals);
    };

    /**
     * @brief The main logger. This logger will print logs with a given format on the output stream.
     * 
     */
    class Logger : public BaseLogger
    {
    public:
        Logger()
        {
        }

        Logger(std::ostream *os, const std::string &format = "") : output_stream_(os), format_(format)
        {
        }

        Logger(std::shared_ptr<std::ostream> os, const std::string &format = "") : output_stream_(os), format_(format)
        {
        }

        /**
         * @brief Set the format of logs.
         * 
         * Use `{}` each time a value must be added. 
         * 
         * Example:
         * 
         * ```cpp
         * Logger logger;
         * logger.setFormat("#> Here, I print my results : result1={}, result2={}");
         * ``` 
         * 
         * @param format the format as a string. 
         */
        void setFormat(const std::string &format)
        {
            this->format_ = format;
        }

        /**
         * @brief Record values.
         * 
         * @tparam TData... the types of input values
         * @param vals the values to log
         */
        template <class... TData>
        void log(TData... vals)
        {
            *this->output_stream_ << fmt::format(this->format_, vals...);
            this->output_stream_->flush();
        }

    protected:
        /** @brief the output stream for logs. */
        std::shared_ptr<std::ostream> output_stream_;

        /** @brief the output format */
        std::string format_;
    };

    /**
     * @brief The standard logger will print logs on the standard output stream.
     * 
     */
    class StdLogger : public Logger
    {
    public:
        StdLogger(const std::string &format = "Not initialized formatter.") : Logger(std::shared_ptr<std::ostream>(&std::cout, [](std::ostream *) {}), format)
        {
        }
    };

    /**
     * @brief The file logger will print logs/data in a file
     * 
     */
    class FileLogger : public Logger
    {
    public:
        FileLogger()
        {
        }

        FileLogger(const std::string &filename, const std::string &format = "") : Logger(new std::ofstream, format)
        {
            std::static_pointer_cast<std::ofstream>(this->output_stream_)->open(filename);
        }

        /**
         * @brief Close the output file stream.
         * 
         */
        void close()
        {
            std::static_pointer_cast<std::ofstream>(this->output_stream_)->close();
        }
    };

    /**
     * @brief The CSV logger will print logs/data in a file with csv format. This logger can be used to save training data.
     * 
     */
    class CSVLogger : public FileLogger
    {
    public:
        CSVLogger(const std::string &filename = "out") : FileLogger(filename + ".csv")
        {
        }

        CSVLogger(const std::string &filename, const std::vector<std::string> &col_names, const std::string &separator = ",") : CSVLogger(filename)
        {
            auto format = tools::repeatString("{}" + separator, col_names.size() - 1) + "{}\n";
            this->setFormat(format);
            for (sdm::size_t i = 0; i < col_names.size(); i++)
            {
                if (i != col_names.size() - 1)
                {
                    *this->output_stream_ << col_names[i] << separator;
                }
                else
                {
                    *this->output_stream_ << col_names[i] << "\n";
                }
            }
        }
    };

    /**
     * @brief The multi logger will print logs in several loggers.
     * 
     */
    class MultiLogger : public BaseLogger, public std::vector<std::shared_ptr<Logger>>
    {
    public:
        MultiLogger(const std::vector<std::shared_ptr<Logger>> &loggers) : std::vector<std::shared_ptr<Logger>>(loggers) {}
        MultiLogger(const std::initializer_list<std::shared_ptr<Logger>> &loggers) : std::vector<std::shared_ptr<Logger>>(loggers) {}


        /**
         * @brief Record values in each loggers.
         * 
         * Go over all sub-loggers and record input values in each of them.
         * 
         * @tparam TData... the types of input values
         * @param vals the values to log
         */
        template <class... TData>
        void log(TData... vals)
        {
            for (auto &logger : *this)
            {
                logger->log(vals...);
            }
        }
    };

} // namespace sdm
