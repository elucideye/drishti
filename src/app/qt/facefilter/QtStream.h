#ifndef QT_STREAM_H
#define QT_STREAM_H

#include <QByteArray>
#include <streambuf>

class QtStream: public std::basic_streambuf<char>
{
public:
    using Base = std::basic_streambuf<char>;
    QtStream(QByteArray& byte_array)
    {
        Base::setg(byte_array.data(), byte_array.data(), byte_array.data() + byte_array.size());
    }
};

#endif // QT_STREAM_H
