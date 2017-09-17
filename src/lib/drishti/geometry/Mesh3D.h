/*! -*-c++-*-
  @file   Mesh3D.h
  @author David Hirvonen
  @brief  3D mesh object.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.

*/

#ifndef __drishti_geometry_Mesh3D_h__
#define __drishti_geometry_Mesh3D_h__

#include "drishti/geometry/drishti_geometry.h"

#include <opencv2/core.hpp>
#include <vector>

DRISHTI_GEOMETRY_BEGIN

template <typename T>
class Mesh3D
{
public:
    using APoint3d = cv::Point3_<T>;
    using APointVec3d = std::vector<APoint3d>;

    Mesh3D() {}
    Mesh3D(const Mesh3D& mesh)
        : m_vertices(mesh.m_vertices)
    {
    }
    Mesh3D(Mesh3D&& mesh)
        : m_vertices(std::move(mesh.m_vertices))
    {
    }

    Mesh3D<T>& operator=(const Mesh3D<T>& src)
    {
        this->m_vertices = src.m_vertices;
        return *this;
    }

    std::size_t size() const { return m_vertices.size(); }

    void push_back(const APoint3d& p) { m_vertices.push_back(p); }

    APoint3d& front() { return m_vertices.front(); }
    const APoint3d& front() const { return m_vertices.front(); }

    typename APointVec3d::iterator begin() { return m_vertices.begin(); }
    typename APointVec3d::iterator end() { return m_vertices.end(); }

protected:
    APointVec3d m_vertices;
};

DRISHTI_GEOMETRY_END

#endif // __drishti_geometry_Mesh3D_h__
