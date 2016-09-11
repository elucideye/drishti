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
  arithmetic.h
  boost_serialize_common.h
  convert.h
  cvmat_serialization.h
  drawing.h
  drishti_core.h
  drishti_csv.h
  drishti_defs.hpp
  drishti_math.h
  drishti_serialize.h
  infix_iterator.h
  padding.h
  serialization.h
  string_utils.h
  timing.h
)
