/*
    This code was taken from the LGPL project OpenMesh.
    Have a look at www.openmesh.org.
*/

#ifndef __VirtualRobot_STLREADER_H__
#define __VirtualRobot_STLREADER_H__


#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/VirtualRobotImportExport.h>
#include <stdio.h>
#include <string>


namespace VirtualRobot
{

    /**
        Implementation of the STL format reader.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT STLReader
    {
    public:
        enum STL_Type { STLA, STLB, NONE };

        // constructor
        STLReader();

        /// Destructor
        virtual ~STLReader() {};


        std::string get_description() const
        {
            return "Stereolithography Interface Format";
        }
        std::string get_extensions() const
        {
            return "stl stla stlb";
        }

        // read data and store it to trimesh
        bool read(const std::string& _filename, TriMeshModelPtr t);

        // read data and store it to trimesh
        bool read(std::istream& _in, STL_Type stltype, TriMeshModelPtr t);

        /** Set the threshold to be used for considering two point to be equal.
            Can be used to merge small gaps */
        void set_epsilon(float _eps)
        {
            eps_ = _eps;
        }

        /// Returns the threshold to be used for considering two point to be equal.
        float epsilon() const
        {
            return eps_;
        }

        void setScaling(float s);

    private:

        int read_int(FILE* _in, bool _swap) const;
        float read_float(FILE* _in, bool _swap = false) const;

        float read_float(std::istream& _in, bool _swap = false) const;
        int read_int(std::istream& _in, bool _swap = false) const;

        STL_Type check_stl_type(const std::string& _filename) const;

        bool read_stla(const std::string& _filename, TriMeshModelPtr t) const;
        bool read_stla(std::istream& _in, TriMeshModelPtr t) const;
        bool read_stlb(const std::string& _filename, TriMeshModelPtr t) const;
        bool read_stlb(std::istream& _in, TriMeshModelPtr t) const;


    private:
        float scaling;
        float eps_;
    };

    typedef boost::shared_ptr<STLReader> STLReaderPtr;
}

#endif
