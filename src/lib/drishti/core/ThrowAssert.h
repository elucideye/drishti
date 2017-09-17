/*! -*-c++-*-
  @file ThrowAssert.h
  @brief Declaration of a throw-exception-on-assertio class.
*/

// Copyright (c) 2015 Softwariness.com (original)
// Copyright (c) 2017 Elucideye Inc. (adaptations)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.

#ifndef __drishti_core_drishti_throw_assert_h__
#define __drishti_core_drishti_throw_assert_h__

#include "drishti/core/drishti_core.h"

#include <exception>
#include <string>
#include <sstream>
#include <iostream>

DRISHTI_CORE_NAMESPACE_BEGIN

/// Exception type for assertion failures
class AssertionFailureException : public std::exception
{
private:
    const char* expression;
    const char* file;
    int line;
    std::string message;
    std::string report;

public:
    /// Helper class for formatting assertion message
    class StreamFormatter
    {
    private:
        std::ostringstream stream;

    public:
        operator std::string() const
        {
            return stream.str();
        }

        template <typename T>
        StreamFormatter& operator<<(const T& value)
        {
            stream << value;
            return *this;
        }
    };

    /// Log error before throwing
    void LogError()
    {
        std::cerr << report << std::endl;
    }

    /// Construct an assertion failure exception
    AssertionFailureException(const char* expression, const char* file, int line, const std::string& message)
        : expression(expression)
        , file(file)
        , line(line)
        , message(message)
    {
        std::ostringstream outputStream;

        if (!message.empty())
        {
            outputStream << message << ": ";
        }

        std::string expressionString = expression;

        if (expressionString == "false" || expressionString == "0" || expressionString == "FALSE")
        {
            outputStream << "Unreachable code assertion";
        }
        else
        {
            outputStream << "Assertion '" << expression << "'";
        }

        outputStream << " failed in file '" << file << "' line " << line;
        report = outputStream.str();

        LogError();
    }

    /// The assertion message
    virtual const char* what() const throw()
    {
        return report.c_str();
    }

    /// The expression which was asserted to be true
    const char* Expression() const throw()
    {
        return expression;
    }

    /// Source file
    const char* File() const throw()
    {
        return file;
    }

    /// Source line
    int Line() const throw()
    {
        return line;
    }

    /// Description of failure
    const char* Message() const throw()
    {
        return message.c_str();
    }

    ~AssertionFailureException() throw()
    {
    }
};

DRISHTI_CORE_NAMESPACE_END

/// Assert that EXPRESSION evaluates to true, otherwise raise AssertionFailureException with associated MESSAGE
/// (which may use C++ stream-style message formatting)
#define drishti_throw_assert(EXPRESSION, MESSAGE)                       \
    if (!(EXPRESSION))                                                  \
    {                                                                   \
        throw drishti::core::AssertionFailureException(#EXPRESSION, __FILE__, __LINE__, (drishti::core::AssertionFailureException::StreamFormatter() << MESSAGE)); \
    }

#endif
