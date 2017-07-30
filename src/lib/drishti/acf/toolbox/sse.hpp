/*******************************************************************************
* Piotr's Image&Video Toolbox      Version 3.23
* Copyright 2013 Piotr Dollar & Ron Appel.  [pdollar-at-caltech.edu]
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/
#ifndef __drishti_acf_toolbox_sse_hpp__
#define __drishti_acf_toolbox_sse_hpp__

#include <iostream>

// Currently assume support for NEON on all ARM platforms
#if defined(__arm64) || defined(__ARM_NEON__) || defined(ANDROID)
// http://codesuppository.blogspot.com/2015/02/sse2neonh-porting-guide-and-header-file.html

#include <arm_neon.h>
#define RETf float32x4_t
#define RETi int32x4_t

//#include "drishti/acf/toolbox/sse2neon.h"

#include "SSE2NEON.h"

#define _mm_load_ss(ptr) (vld1q_dup_f32((ptr)))
#define _mm_setzero_ps() (vmovq_n_f32(0.f))
#define _mm_set1_ps(a) (vmovq_n_f32((a)))
#define _mm_set1_epi32(a) (vmovq_n_s32((a))) // CHECK
#define _mm_loadu_ps(ptr) (vld1q_f32((ptr)))
#define _mm_load_ps(ptr) (vld1q_f32((ptr)))
#define _mm_storeu_ps(ptr, a) (vst1q_f32((ptr), (a)))
#define _mm_store_ps(ptr, a) (vst1q_f32((ptr), (a)))
#define _mm_store_ss(ptr, a) ((ptr)[0] = a[0]) // CHECK
#define _mm_add_ps(a, b) (vaddq_f32((a), (b)))
#define _mm_sub_ps(a, b) (vsubq_f32((a), (b)))
#define _mm_mul_ps(a, b) (vmulq_f32((a), (b)))

// http://clb.demon.fi/MathGeoLib/nightly/docs/simd.h_code.html
// inline RETf _mm_rsqrt_ps(const RETf &val)
// {
//     RETf e = vrsqrteq_f32(val);
//     e = vmulq_f32(vrsqrtsq_f32(vmulq_f32(e, e), val), e);
//     e = vmulq_f32(vrsqrtsq_f32(vmulq_f32(e, e), val), e);
//     return e;
// }

#define add_ps vaddq_f32
#define sub_ps vsubq_f32
#define mul_ps vmulq_f32
#define div_ps(a, b) ((a) / (b))
#define min_ps vminq_f32
#define max_ps vmaxq_f32
#define s4f_to_s4i(s4f) vreinterpretq_u32_f32((s4f))
#define s4i_to_s4f(s4i) vreinterpretq_f32_u32((s4i))
#define and_ps(x, y) s4i_to_s4f(vandq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define andnot_ps(x, y) s4i_to_s4f(vbicq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define or_ps(x, y) s4i_to_s4f(vorrq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define xor_ps(x, y) s4i_to_s4f(veorq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define ornot_ps(x, y) s4i_to_s4f(vornq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define _mm_xor_ps(x, y) xor_ps(x, y)

inline std::ostream& operator<<(std::ostream& os, const RETf& v)
{
    os << '{' << float(v[3]) << ',' << float(v[2]) << ',' << float(v[1]) << ',' << float(v[0]) << '}';
    return os;
}

#else
#include <emmintrin.h> // SSE2:<e*.h>, SSE3:<p*.h>, SSE4:<s*.h>
#define RETf __m128
#define RETi __m128i

inline std::ostream& operator<<(std::ostream& os, const RETf& m)
{
    float v[4];
    _mm_storeu_ps(v, m);
    os << '{' << v[3] << ',' << v[2] << ',' << v[1] << ',' << v[0] << '}';
    return os;
}

#endif

// set, load and store values
inline RETf SET(const float& x)
{
    return _mm_set1_ps(x);
}
inline RETf SET(float x, float y, float z, float w)
{
    return _mm_set_ps(x, y, z, w);
}
inline RETi SET(const int& x)
{
    return _mm_set1_epi32(x);
}

//RETf LD( const float &x ) { return _mm_load_ps(&x); }
inline RETf LD(const float& x)
{
    return _mm_loadu_ps(&x);
}
inline RETf LDu(const float& x)
{
    return _mm_loadu_ps(&x);
}

inline RETf STR(float& x, const RETf y)
{
    _mm_store_ps(&x, y); // Stores four single-precision, floating-point values
    return y;
}
inline RETf STR1(float& x, const RETf y)
{
    _mm_store_ss(&x, y); // Stores the lower single-precision, floating-point value
    return y;
}
inline RETf STRu(float& x, const RETf y)
{
    _mm_storeu_ps(&x, y);
    return y;
}
inline RETf STR(float& x, const float y)
{
    return STR(x, SET(y));
}

// arithmetic operators
inline RETi ADD(const RETi x, const RETi y)
{
    return _mm_add_epi32(x, y);
}
inline RETf ADD(const RETf x, const RETf y)
{
    return _mm_add_ps(x, y);
}
inline RETf ADD(const RETf x, const RETf y, const RETf z)
{
    return ADD(ADD(x, y), z);
}
inline RETf ADD(const RETf a, const RETf b, const RETf c, const RETf& d)
{
    return ADD(ADD(ADD(a, b), c), d);
}
inline RETf SUB(const RETf x, const RETf y)
{
    return _mm_sub_ps(x, y);
}
inline RETf MUL(const RETf x, const RETf y)
{
    return _mm_mul_ps(x, y);
}
inline RETf MUL(const RETf x, const float y)
{
    return MUL(x, SET(y));
}
inline RETf MUL(const float x, const RETf y)
{
    return MUL(SET(x), y);
}
inline RETf INC(RETf& x, const RETf y)
{
    return x = ADD(x, y);
}
inline RETf INC(float& x, const RETf y)
{
    RETf t = ADD(LD(x), y);
    return STR(x, t);
}
inline RETf DEC(RETf& x, const RETf y)
{
    return x = SUB(x, y);
}
inline RETf DEC(float& x, const RETf y)
{
    RETf t = SUB(LD(x), y);
    return STR(x, t);
}
inline RETf MIN_sse(const RETf x, const RETf y)
{
    return _mm_min_ps(x, y);
}
inline RETf MAX_sse(const RETf x, const RETf y)
{
    return _mm_max_ps(x, y);
}
inline RETf RCP(const RETf x)
{
    return _mm_rcp_ps(x);
}
inline RETf RCPSQRT(const RETf x)
{
    return _mm_rsqrt_ps(x);
}

// logical operators
inline RETf AND(const RETf x, const RETf y)
{
    return _mm_and_ps(x, y);
}
inline RETi AND(const RETi x, const RETi y)
{
    return _mm_and_si128(x, y);
}
inline RETf ANDNOT(const RETf x, const RETf y)
{
    return _mm_andnot_ps(x, y);
}
inline RETf OR(const RETf x, const RETf y)
{
    return _mm_or_ps(x, y);
}
inline RETf XOR(const RETf x, const RETf y)
{
    return _mm_xor_ps(x, y);
}

// comparison operators
inline RETf CMPGT(const RETf x, const RETf y)
{
    return _mm_cmpgt_ps(x, y);
}
inline RETf CMPLT(const RETf x, const RETf y)
{
    return _mm_cmplt_ps(x, y);
}
inline RETi CMPGT(const RETi x, const RETi y)
{
    return _mm_cmpgt_epi32(x, y);
}
inline RETi CMPLT(const RETi x, const RETi y)
{
    return _mm_cmplt_epi32(x, y);
}

// conversion operators
inline RETf CVT(const RETi x)
{
    return _mm_cvtepi32_ps(x);
}
inline RETi CVT(const RETf x)
{
    return _mm_cvttps_epi32(x);
}

#undef RETf
#undef RETi
#endif
