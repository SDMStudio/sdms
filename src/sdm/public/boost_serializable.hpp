#pragma once

#include <regex>
#include <iostream>
#include <fstream>     

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

template <class SerializableClass>
class BoostSerializable
{
public:
    /**
     * @brief Save a value function into a file. 
     * The extension of the file will indicate the type of formatage for recording (`.txt` = text format, '.xml' = XML format, other = binary format). 
     * 
     * @param filename the filename 
     */
    void save(std::string filename)
    {
        std::ofstream ofs(filename);
        if (regex_match(filename, std::regex(".*\\.txt$")) || regex_match(filename, std::regex(".*\\.TXT$")))
        {
            boost::archive::text_oarchive output_archive(ofs);
            this->serialize(output_archive, 0);
            // else if (regex_match(filename, std::regex(".*\\.xml$")) || regex_match(filename, std::regex(".*\\.XML$")))
            // {
            //     boost::archive::xml_oarchive output_archive(ofs);
            //     this->serialize(output_archive, 0);
            // }
        }
        else
        {
            boost::archive::binary_oarchive output_archive(ofs);
            this->serialize(output_archive, 0);
        }
        ofs.close();
    }

    /**
     * @brief Load a value function from a file.
     * The extension of the file will indicate the type of formatage for reading (`.txt` = text format, '.xml' = XML format, other = binary format). 
     * 
     * @param filename the filename 
     */
    void load(std::string filename)
    {
        std::ifstream ifs(filename);
        if (regex_match(filename, std::regex(".*\\.txt$")) || regex_match(filename, std::regex(".*\\.TXT$")))
        {
            boost::archive::text_iarchive input_archive(ifs);
            this->serialize(input_archive, 0);
            // else if (regex_match(filename, std::regex(".*\\.xml$")) || regex_match(filename, std::regex(".*\\.XML$")))
            // {
            //     boost::archive::xml_iarchive input_archive(ifs);
            //     this->serialize(input_archive, 0);
            // }
        }
        else
        {
            boost::archive::binary_iarchive input_archive(ifs);
            this->serialize(input_archive, 0);
        }
        ifs.close();
    }

protected:
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &archive, const unsigned int version)
    {
        static_cast<SerializableClass *>(this)->serialize(archive, version);
    }
};