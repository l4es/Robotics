/*===========================================================================*\
 *                                                                           *
 *                               OpenMesh                                    *
 *      Copyright (C) 2001-2014 by Computer Graphics Group, RWTH Aachen      *
 *                           www.openmesh.org                                *
 *                                                                           *
 *---------------------------------------------------------------------------*
 *  This file is part of OpenMesh.                                           *
 *                                                                           *
 *  OpenMesh is free software: you can redistribute it and/or modify         *
 *  it under the terms of the GNU Lesser General Public License as           *
 *  published by the Free Software Foundation, either version 3 of           *
 *  the License, or (at your option) any later version with the              *
 *  following exceptions:                                                    *
 *                                                                           *
 *  If other files instantiate templates or use macros                       *
 *  or inline functions from this file, or you compile this file and         *
 *  link it with other files to produce an executable, this file does        *
 *  not by itself cause the resulting executable to be covered by the        *
 *  GNU Lesser General Public License. This exception does not however       *
 *  invalidate any other reasons why the executable file might be            *
 *  covered by the GNU Lesser General Public License.                        *
 *                                                                           *
 *  OpenMesh is distributed in the hope that it will be useful,              *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU Lesser General Public License for more details.                      *
 *                                                                           *
 *  You should have received a copy of the GNU LesserGeneral Public          *
 *  License along with OpenMesh.  If not,                                    *
 *  see <http://www.gnu.org/licenses/>.                                      *
 *                                                                           *
\*===========================================================================*/



#include "STLReader.h"
#include <VirtualRobot/XML/BaseIO.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>

// STL
#include <map>

#include <float.h>
#include <fstream>

namespace VirtualRobot
{



    STLReader::
    STLReader()
        : eps_(FLT_MIN)
    {
        scaling = 1.0f;
    }


    //-----------------------------------------------------------------------------


    bool
    STLReader::
    read(const std::string& _filename, TriMeshModelPtr t)
    {
        bool result = false;

        STL_Type file_type = NONE;

        if (_filename.length() >= 4)
        {
            std::string ending = _filename.substr(_filename.length() - 4, 4);
            BaseIO::getLowerCase(ending);

            if (ending == ".stl")
            {
                file_type = check_stl_type(_filename);
            }
            else if (ending == "stla")
            {
                file_type = STLA;
            }
            else if (ending == "stlb")
            {
                file_type = STLB;
            }
        }

        switch (file_type)
        {
            case STLA:
            {
                result = read_stla(_filename, t);
                break;
            }

            case STLB:
            {
                result = read_stlb(_filename, t);
                break;
            }

            default:
            {
                result = false;
                break;
            }
        }


        return result;
    }

    bool
    STLReader::read(std::istream& _is, STL_Type stltype, TriMeshModelPtr t)
    {

        bool result = false;

        if (stltype == STLB)
        {
            result = read_stlb(_is, t);
        }
        else
        {
            result = read_stla(_is, t);
        }

        return result;
    }


    //-----------------------------------------------------------------------------


#ifndef DOXY_IGNORE_THIS

    class CmpVec
    {
    public:

        CmpVec(float _eps = FLT_MIN) : eps_(_eps) {}

        bool operator()(const Eigen::Vector3f& _v0, const Eigen::Vector3f& _v1) const
        {
            if (fabs(_v0[0] - _v1[0]) <= eps_)
            {
                if (fabs(_v0[1] - _v1[1]) <= eps_)
                {
                    return (_v0[2] < _v1[2] - eps_);
                }
                else
                {
                    return (_v0[1] < _v1[1] - eps_);
                }
            }
            else
            {
                return (_v0[0] < _v1[0] - eps_);
            }
        }

    private:
        float eps_;
    };

#endif


    //-----------------------------------------------------------------------------

    void trimStdString(std::string& _string)
    {
        // Trim Both leading and trailing spaces

        size_t start = _string.find_first_not_of(" \t\r\n");
        size_t end   = _string.find_last_not_of(" \t\r\n");

        if ((std::string::npos == start) || (std::string::npos == end))
        {
            _string = "";
        }
        else
        {
            _string = _string.substr(start, end - start + 1);
        }
    }

    //-----------------------------------------------------------------------------

    bool
    STLReader::
    read_stla(const std::string& _filename, TriMeshModelPtr t) const
    {
        std::fstream in(_filename.c_str(), std::ios_base::in);

        if (!in)
        {
            VR_ERROR << "[STLReader] : cannot not open file "
                     << _filename
                     << std::endl;
            return false;
        }

        bool res = read_stla(in, t);

        if (in)
        {
            in.close();
        }

        return res;
    }

    //-----------------------------------------------------------------------------

    bool
    STLReader::
    read_stla(std::istream& _in, TriMeshModelPtr t) const
    {

        if (!t)
        {
            return false;
        }

        unsigned int               i;
        Eigen::Vector3f            v;
        Eigen::Vector3f            n;
        std::vector<unsigned int> vhandles;

        CmpVec comp(eps_);
        std::map<Eigen::Vector3f, unsigned int, CmpVec>            vMap(comp);
        std::map<Eigen::Vector3f, unsigned int, CmpVec>::iterator  vMapIt;

        std::string line;

        bool facet_normal(false);

        while (_in && !_in.eof())
        {

            // Get one line
            std::getline(_in, line);

            if (_in.bad())
            {
                VR_ERROR << "  Warning! Could not read stream properly!\n";
                return false;
            }

            // Trim Both leading and trailing spaces
            trimStdString(line);

            // Normal found?
            if (line.find("facet normal") != std::string::npos)
            {
                std::stringstream strstream(line);

                std::string garbage;

                // facet
                strstream >> garbage;

                // normal
                strstream >> garbage;

                strstream >> n[0];
                strstream >> n[1];
                strstream >> n[2];

                facet_normal = true;
            }

            // Detected a triangle
            if ((line.find("outer") != std::string::npos) || (line.find("OUTER") != std::string::npos))
            {

                vhandles.clear();

                for (i = 0; i < 3; ++i)
                {
                    // Get one vertex
                    std::getline(_in, line);
                    trimStdString(line);

                    std::stringstream strstream(line);

                    std::string garbage;
                    strstream >> garbage;

                    strstream >> v[0];
                    strstream >> v[1];
                    strstream >> v[2];

                    v *= scaling;

                    // has vector been referenced before?
                    if ((vMapIt = vMap.find(v)) == vMap.end())
                    {
                        // No : add vertex and remember idx/vector mapping
                        int handle = int (t->vertices.size());
                        t->addVertex(v);//_bi.add_vertex(v);
                        vhandles.push_back(handle);
                        vMap[v] = handle;
                    }
                    else
                        // Yes : get index from map
                    {
                        vhandles.push_back(vMapIt->second);
                    }
                }

                // Add face only if it is not degenerated
                if ((vhandles[0] != vhandles[1]) &&
                    (vhandles[0] != vhandles[2]) &&
                    (vhandles[1] != vhandles[2]))
                {
                    MathTools::TriangleFace f;
                    f.set(vhandles[0], vhandles[1], vhandles[2]);

                    if (facet_normal)
                    {
                        unsigned int noId = t->normals.size();
                        t->addNormal(n);
                        f.setNormal(noId, noId, noId);
                    }

                    int fh = int(t->faces.size());
                    t->addFace(f);
                    //_bi.add_face(vhandles);
                    facet_normal = false;
                }
            }
        }

        return true;
    }

    //-----------------------------------------------------------------------------

    bool
    STLReader::
    read_stlb(const std::string& _filename, TriMeshModelPtr t) const
    {
        std::fstream in(_filename.c_str(), std::ios_base::in | std::ios_base::binary);

        if (!in)
        {
            VR_ERROR << "[STLReader] : cannot not open file "
                     << _filename
                     << std::endl;
            return false;
        }

        bool res = read_stlb(in, t);

        if (in)
        {
            in.close();
        }

        return res;
    }

    //-----------------------------------------------------------------------------



    float STLReader::read_float(std::istream& _in, bool _swap) const
    {
        union u3
        {
            float f;
            unsigned char c[4];
        } fc;
        _in.read((char*)fc.c, 4);

        if (_swap)
        {
            std::swap(fc.c[0], fc.c[3]);
            std::swap(fc.c[1], fc.c[2]);
        }

        return fc.f;
    }

    int STLReader::read_int(std::istream& _in, bool _swap) const
    {
        union u2
        {
            int i;
            unsigned char c[4];
        } ic;
        _in.read((char*)ic.c, 4);

        if (_swap)
        {
            std::swap(ic.c[0], ic.c[3]);
            std::swap(ic.c[1], ic.c[2]);
        }

        return ic.i;
    }

    bool
    STLReader::
    read_stlb(std::istream& _in, TriMeshModelPtr t) const
    {
        if (!t)
        {
            return false;
        }

        char                       dummy[100];
        bool                       swapFlag;
        unsigned int               i, nT;
        Eigen::Vector3f            v, n;
        std::vector<unsigned int> vhandles;


        std::map<Eigen::Vector3f, unsigned int, CmpVec>  vMap;
        std::map<Eigen::Vector3f, unsigned int, CmpVec>::iterator vMapIt;


        // check size of types
        if ((sizeof(float) != 4) || (sizeof(int) != 4))
        {
            VR_ERROR << "[STLReader] : wrong type size\n";
            return false;
        }

        // determine endian mode
        union
        {
            unsigned int i;
            unsigned char c[4];
        } endian_test;
        endian_test.i = 1;
        swapFlag = (endian_test.c[3] == 1);

        // read number of triangles
        _in.read(dummy, 80);
        nT = read_int(_in, swapFlag);

        // read triangles
        while (nT)
        {
            vhandles.clear();

            // read triangle normal
            n[0] = read_float(_in, swapFlag);
            n[1] = read_float(_in, swapFlag);
            n[2] = read_float(_in, swapFlag);

            // triangle's vertices
            for (i = 0; i < 3; ++i)
            {
                v[0] = read_float(_in, swapFlag);
                v[1] = read_float(_in, swapFlag);
                v[2] = read_float(_in, swapFlag);

                v *= scaling;

                // has vector been referenced before?
                if ((vMapIt = vMap.find(v)) == vMap.end())
                {
                    // No : add vertex and remember idx/vector mapping
                    unsigned int handle = t->vertices.size();
                    t->addVertex(v);
                    //_bi.add_vertex(v);
                    vhandles.push_back(handle);
                    vMap[v] = handle;
                }
                else
                    // Yes : get index from map
                {
                    vhandles.push_back(vMapIt->second);
                }
            }


            // Add face only if it is not degenerated
            if ((vhandles[0] != vhandles[1]) &&
                (vhandles[0] != vhandles[2]) &&
                (vhandles[1] != vhandles[2]))
            {
                MathTools::TriangleFace f;
                f.set(vhandles[0], vhandles[1], vhandles[2]);
                //if (facet_normal) {
                unsigned int noId = t->normals.size();
                t->addNormal(n);
                f.setNormal(noId, noId, noId);
                //}
                int fh = int(t->faces.size());
                t->addFace(f);
                //FaceHandle fh = _bi.add_face(vhandles);
            }

            _in.read(dummy, 2);
            --nT;
        }

        return true;
    }

    //-----------------------------------------------------------------------------

    int STLReader::read_int(FILE* _in, bool _swap) const
    {
        union u2
        {
            int i;
            unsigned char c[4];
        } ic;
        size_t bytesRead = fread((char*)ic.c, 1, 4, _in);

        if (_swap)
        {
            std::swap(ic.c[0], ic.c[3]);
            std::swap(ic.c[1], ic.c[2]);
        }

        return ic.i;
    }

    float STLReader::read_float(FILE* _in, bool _swap) const
    {
        union u3
        {
            float f;
            unsigned char c[4];
        } fc;
        size_t bytesRead = fread((char*)fc.c, 1, 4, _in);

        if (_swap)
        {
            std::swap(fc.c[0], fc.c[3]);
            std::swap(fc.c[1], fc.c[2]);
        }

        return fc.f;
    }

    STLReader::STL_Type
    STLReader::
    check_stl_type(const std::string& _filename) const
    {
        // assume it's binary stl, then file size is known from #triangles
        // if size matches, it's really binary

        // open file
        FILE* in = fopen(_filename.c_str(), "rb");

        if (!in)
        {
            return NONE;
        }

        // determine endian mode
        union
        {
            unsigned int i;
            unsigned char c[4];
        } endian_test;
        endian_test.i = 1;
        bool swapFlag = (endian_test.c[3] == 1);

        // read number of triangles
        char dummy[100];
        size_t bytesRead = fread(dummy, 1, 80, in);
        size_t nT = read_int(in, swapFlag);


        // compute file size from nT
        size_t binary_size = 84 + nT * 50;


        // get actual file size
        size_t file_size(0);
        rewind(in);

        while (!feof(in))
        {
            file_size += fread(dummy, 1, 100, in);
        }

        fclose(in);


        // if sizes match -> it's STLB
        return (binary_size == file_size ? STLB : STLA);
    }

    void STLReader::setScaling(float s)
    {
        scaling = s;
    }



    //=============================================================================
} // namespace

