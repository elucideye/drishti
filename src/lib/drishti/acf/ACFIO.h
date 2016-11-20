/*!
  @file   ACFIO.h
  @author David Hirvonen (C++ implementation)
  @brief  Declaration of deserialization routines for ACFIO mat models.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __DRISHTI__ACFIO__
#define __DRISHTI__ACFIO__

#include "drishti/acf/drishti_acf.h"
#include "drishti/acf/ACFField.h"

#include "drishti/core/drishti_defs.hpp"
#include "drishti/core/IndentingOStreamBuffer.h"

#include <opencv2/core/core.hpp>

// TODO: Work on making these private but also sharable
#include "cvmatio/MatlabIO.hpp"
#include "cvmatio/MatlabIOContainer.hpp"

// Boost serialization:
#include "drishti/core/serialization.h"

#include <string>
#include <iostream>

DRISHTI_ACF_NAMESPACE_BEGIN

typedef std::vector< MatlabIOContainer > VecContainer;
typedef std::vector< VecContainer > VecVecContainer;

template <typename T2>
std::ostream& operator<<(std::ostream &os, const Field<T2>& src)
{
    os << src.name << ":";
    if(src.has)
    {
        if(!src.isLeaf)
        {
            os << "\n{\n";
            {
                IndentingOStreambuf indent(os);
                os << src.value;
            }
            os << "\n}";

        }
        else
        {
            os << src.value ;
        }
    }
    else
    {
        os << "[]";
    }
    return os;
}

template <typename T2>
std::ostream& operator<<(std::ostream &os, const Field<std::vector<T2>> &src)
{
    os << src.name << ":";
    if(src.has)
    {
        for(int i = 0; i < src.value.size()-1; i++)
        {
            os << src.value[i] << ",";
        }

        os << src.value.back() ;
    }
    else
    {
        os << "[]";
    }
    return os;
}

template <typename T1, typename T2> struct Cast
{
    T2 operator()(const T1 &t1)
    {
        return T2(t1);
    }
};

template <typename T1, typename T2>
struct Cast<std::vector<T1>, std::vector<T2>>
{
    std::vector<T2> operator()(const std::vector<T1> &t1)
    {
        return std::vector<T2>(t1.begin(), t1.end());
    }
};

template <typename T1, typename T2>
struct Cast<std::vector<T1>, Field<std::vector<T2>> >
{
    std::vector<T2> operator()(const std::vector<T1> &t1)
    {
        return std::vector<T2>(t1.begin(), t1.end());
    }
};

template <typename T1, typename T2=T1, typename C=Cast<T1,T2> > struct Finder
{
    // Handle all numbers:
    bool operator()(MatlabIO &matio, std::vector<MatlabIOContainer>& variables, const std::string &name, T2 &value)
    {
        bool status = false;
        cv::Mat src = matio.find<cv::Mat>(variables, name);
        if(src.size().area() > 0)
        {
            status = true;
            T1 stage = src.at<T1>(0,0);
            value = C()(stage);
        }
        return status;
    }

    // Handle strings and other miscellaneous types;

};

template <> struct Finder<cv::Mat, cv::Mat>
{
    bool operator()(MatlabIO &matio, std::vector<MatlabIOContainer>& variables, const std::string &name, cv::Mat &value)
    {
        value = matio.find<cv::Mat>(variables, name);
        return !value.empty();
    }
};

template <> struct Finder<std::string, std::string >
{
    bool operator()(MatlabIO &matio, std::vector<MatlabIOContainer>& variables, const std::string &name, std::string &value)
    {
        value = matio.find<std::string>(variables, name);
        return value.size();
    }
};

template <> struct Finder<cv::Size, cv::Size >
{
    bool operator()(MatlabIO &matio, std::vector<MatlabIOContainer>& variables, const std::string &name, cv::Size &value)
    {
        bool status = false;
        cv::Mat src = matio.find<cv::Mat>(variables, name);
        if(src.size().area() > 0)
        {
            status = true;
            value = { int(src.at<double>(0,0)), int(src.at<double>(0,1)) };
        }
        return status;
    }
};

// Specialize for vectors:
template <typename T1, typename T2> struct Finder<std::vector<T1>, std::vector<T2>>
{
    bool operator()(MatlabIO &matio, std::vector<MatlabIOContainer>& variables, const std::string &name, std::vector<T2> &value)
    {
        bool status = false;
        cv::Mat src = matio.find<cv::Mat>(variables, name);
        if(src.size().area() > 0)
        {
            status = true;
            std::copy(src.begin<T1>(), src.end<T1>(), std::back_inserter(value));
        }
        return status;
    }
};

template <typename T1, typename T2 > struct Finder<Field<T1>, Field<T2> >
{
    bool operator()(MatlabIO &matio, std::vector<MatlabIOContainer>& variables, const std::string &name, Field<T2> &value)
    {
        T1 tmp;
        bool status = Finder<T1,T1>()(matio, variables, name, tmp);
        value.set(name, status, true, Cast<T1,T2>()(tmp));
        return status;
    }
};

#define DO_DEBUG_LOAD 0

#define USE_INDENTING_STREAM 0

template <typename T>
struct ParserNode
{
    ParserNode() {}

    ParserNode(const ParserNode &node)
        : m_name(node.m_name)
        , m_object(node.m_object)
        , m_variables(node.m_variables)
        , m_indent(node.m_indent)  {}

    ParserNode(const std::string &filename, T &object)
        : m_name("root")
        , m_object(&object)
    {
#if USE_INDENTING_STREAM
        m_indent = std::make_shared<IndentingOStreambuf>(std::cout);
#endif
        open(filename);
        log();
    }

    ParserNode(const std::string &name, T &object, VecContainer &variables)
        : m_name(name)
        , m_object(&object)
        , m_variables(variables)
    {
#if USE_INDENTING_STREAM
        m_indent = std::make_shared<IndentingOStreambuf>(std::cout);
#endif
        log();
    }

    // Istream input
    ParserNode(std::istream &is, T &object)
        : m_name("root")
        , m_object(&object)
    {
#if USE_INDENTING_STREAM
        m_indent = std::make_shared<IndentingOStreambuf>(std::cout);
#endif
        open(is);
        log();
    }

    ParserNode & operator=(const ParserNode &src)
    {
        m_name = src.m_name;
        m_object = src.m_object;
        m_variables = src.m_variables;
        m_indent = src.m_indent;
        return (*this);
    }

    void log()
    {
#if DO_DEBUG_LOAD
        std::cout << "====" << m_name << "====" << std::endl;
        for(auto &v : m_variables)
        {
            std::cout << v.name() << " " << v.type() << std::endl;
        }
#endif
    }
    int open(std::istream &is)
    {
        // create a new reader
        bool ok = m_matio.attach(is);
        if (!ok)
        {
            return -1;
        }

        // read all of the variables in the file
        m_variables = m_matio.read();

        return 0;        
    }

    int open(const std::string &filename)
    {
        // create a new reader
        bool ok = m_matio.open(filename, "r");
        if (!ok)
        {
            return -1;
        }

        // read all of the variables in the file
        m_variables = m_matio.read();
        m_matio.close();

        return 0;
    }

    // Return single instance
    template <typename T2>
    ParserNode<T2> create(const std::string &name, T2 &object)
    {
        return ParserNode<T2>(name, object, m_matio.find< VecVecContainer >(m_variables, name)[0]);
    }

    template <typename T2>
    ParserNode<Field<T2>> create(const std::string &name, Field<T2> &object)
    {
        object.set(name);
        object.mark(true);
        return ParserNode<Field<T2>>(name, object, m_matio.find< VecVecContainer >(m_variables, name)[0]);
    }

    // Return single instance
    template <typename T2>
    std::vector<ParserNode<Field<typename T2::value_type>>> createVec(const std::string &name, Field<T2> &object)
    {

    }


    T& operator()()
    {
        return m_object;
    }

    template <typename T1, typename T2 >
    bool parse(const std::string &name, T2 &value)
    {
        Finder<T1, T2>()(m_matio, m_variables, name, value);
        return true;
    }

    T& operator *()
    {
        return *m_object;
    }
    const T& operator *() const
    {
        return *m_object;
    }

    T* operator->()
    {
        return m_object;
    }
    const T* operator->() const
    {
        return m_object;
    }

    T & get()
    {
        return *m_object;
    }

    std::string m_name;
    T *m_object;
    VecContainer m_variables;

    std::shared_ptr<IndentingOStreambuf> m_indent;

    MatlabIO m_matio;
};

DRISHTI_ACF_NAMESPACE_END

#endif /* defined(__DRISHTI__ACFIO__) */
