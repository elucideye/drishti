

#ifndef __drishti_qt_facefilter_QtStream_h__
#define __drishti_qt_facefilter_QtStream_h__

#include <QByteArray>
#include <streambuf>

class QtStream : public std::basic_streambuf<char>
{
public:
    using Base = std::basic_streambuf<char>;
    QtStream(QByteArray& byte_array)
    {
        Base::setg(byte_array.data(), byte_array.data(), byte_array.data() + byte_array.size());
    }
};

#endif // __drishti_qt_facefilter_QtStream_h__
