#pragma once

#include <iostream>
#include <locale>

#include <fmt/format.h>
#include <sdm/tools.hpp>

namespace sdm
{

    class BaseLogger
    {
    public:
        template <class... TData>
        void log(TData... vals);
    };

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

        void setFormat(const std::string &format)
        {
            this->format_ = format;
        }

        template <class... TData>
        void log(TData... vals)
        {
            *this->output_stream_ << fmt::format(this->format_, vals...);
            this->output_stream_->flush();
        }

    protected:
        std::shared_ptr<std::ostream> output_stream_;
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
     * @brief The multi logger will print logs from several loggers.
     * 
     */
    class MultiLogger : public BaseLogger, public std::vector<std::shared_ptr<Logger>>
    {
    public:
        MultiLogger(const std::vector<std::shared_ptr<Logger>> &loggers) : std::vector<std::shared_ptr<Logger>>(loggers) {}
        MultiLogger(const std::initializer_list<std::shared_ptr<Logger>> &loggers) : std::vector<std::shared_ptr<Logger>>(loggers) {}

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
