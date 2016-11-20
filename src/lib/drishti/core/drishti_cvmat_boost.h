/*!
  @file   drishti_cvmat_boost.h
  @author Cristoph Heindl
  @brief  Private header for boost::serialize cv::Mat (de)serialization

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

// (c) 2011 Christoph Heindl
// https://cheind.wordpress.com/2011/12/06/serialization-of-cvmat-objects-using-boost/
//
// (c) 2013 Konstantin Sorokin
// https://github.com/thekvs/imgdupl-opencv/blob/master/serialize.hpp

*/

#ifndef __DRISHTI_CVMAT_BOOST_HPP_INCLUDED__
#define __DRISHTI_CVMAT_BOOST_HPP_INCLUDED__

#include "drishti/core/drishti_core.h"
#include <opencv2/opencv.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)

// Macros fix some IDE formatting

DRISHTI_BEGIN_NAMESPACE(boost)
DRISHTI_BEGIN_NAMESPACE(serialization)

/** Serialization support for cv::Mat */
template<class Archive>
void save(Archive &ar, const cv::Mat &m, const unsigned int BOOST_ATTRIBUTE_UNUSED version)
{
    size_t elem_size = m.elemSize();
    size_t elem_type = m.type();

    ar & m.cols;
    ar & m.rows;
    ar & elem_size;
    ar & elem_type;

    const size_t data_size = m.cols * m.rows * elem_size;
    ar & boost::serialization::make_array(m.ptr(), data_size);
}

/** Serialization support for cv::Mat */
template<class Archive>
void load(Archive &ar, cv::Mat &m, const unsigned int BOOST_ATTRIBUTE_UNUSED version)
{
    int    cols, rows;
    size_t elem_size, elem_type;

    ar & cols;
    ar & rows;
    ar & elem_size;
    ar & elem_type;

    m.create(rows, cols, int(elem_type));

    size_t data_size = m.cols * m.rows * elem_size;
    ar & boost::serialization::make_array(m.ptr(), data_size);
}

DRISHTI_END_NAMESPACE(serialization)
DRISHTI_END_NAMESPACE(boost)

#endif // __DRISHTI_CVMAT_BOOST_HPP_INCLUDED__
