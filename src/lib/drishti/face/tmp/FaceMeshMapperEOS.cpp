/*! -*-c++-*-
  @file   FaceMeshMapperEOS.cpp
  @author David Hirvonen (from original code by Patrik Huber)
  @brief  Declaration of a FaceMeshMapper interface to the EOS library.

  This is based on sample code provided with the EOS library.
  See: https://github.com/patrikhuber/eos

  \copyright Copyright 2014-2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_FaceMeshMapperEOS_h__
#define __drishti_face_FaceMeshMapperEOS_h__ 1


class FaceMeshContainerEOS : public FaceMeshContainer
{
public:
    FaceMeshContainerEOS() = default;
    ~FaceMeshContainerEOS() = default;
    
    virtual drishti::face::FaceMesh getFaceMesh();
    virtual cv::Point3f getRotation(); // Euler: pitch, yaw, roll
    virtual cv::Mat extractTexture(const cv::Mat& image);
    virtual void serialize(const std::string &filename);
    virtual void drawWireFrame(cv::Mat &iso);    
    virtual void drawWireFrameOnIso(cv::Mat &iso);

protected:
    
};

#endif // __drishti_face_FaceMeshMapperEOS_h__
