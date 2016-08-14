/*!
  @file   FetchResource.h
  @author David Hirvonen
  @brief  Declaration of a C API to fetch system specific resources.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

//
//  FetchResource.m
//  drishtisdk
//
//  Created by David Hirvonen on 2/26/16.
//
//

#ifndef FETCH_RESOURCE_H
#define FETCH_RESOURCE_H

#include <string>
std::string getResourcePath(const std::string &resource);

#endif // FETCH_RESOURCE_H
