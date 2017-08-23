#ifndef _facefilter_gl_version_h_
#define _facefilter_gl_version_h_

struct GLVersion
{
    GLVersion() = default;
    GLVersion(int major, int minor) : major(major), minor(minor) {}
    int major = 2;
    int minor = 0;
};

#endif // _facefilter_gl_version_h_
