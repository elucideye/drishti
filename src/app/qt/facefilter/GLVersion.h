#ifndef _facefilter_gl_version_h_
#define _facefilter_gl_version_h_

#if defined(major)
#undef major
#endif

#if defined(minor)
#undef minor
#endif

struct GLVersion
{
    GLVersion() = default;
    GLVersion(int major, int minor)
        : major(major)
        , minor(minor)
    {
    }
    int major = 2;
    int minor = 0;
};

#endif // _facefilter_gl_version_h_
