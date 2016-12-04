# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED LIB_CORE_CORE_SUGAR_CMAKE_)
  return()
else()
  set(LIB_CORE_CORE_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

sugar_files(DRISHTI_CORE_SRCS
  Logger.cpp
  Shape.cpp
  ThreadPool.cpp
  arithmetic.cpp
  convert.cpp
  drawing.cpp
  padding.cpp
  string_utils.cpp
)

# For now make them all public
sugar_files(DRISHTI_CORE_HDRS_PUBLIC
  Field.h
  FixedField.h
  IndentingOStreamBuffer.h
  Line.h
  Logger.h
  Parallel.h
  Shape.h
  ThreadPool.h
  arithmetic.h
  boost_serialize_common.h
  convert.h
  drawing.h
  drishti_algorithm.h
  drishti_cereal_pba.h
  drishti_core.h
  drishti_csv.h
  drishti_cv_boost.h
  drishti_cv_cereal.h    
  drishti_cvmat_boost.h
  drishti_cvmat_cereal.h  
  drishti_defs.hpp
  drishti_math.h
  drishti_operators.h
  drishti_serialize.h
  drishti_stdlib_string.h
  drishti_string_hash.h
  infix_iterator.h
  make_unique.h
  padding.h
  string_utils.h
  timing.h
)
