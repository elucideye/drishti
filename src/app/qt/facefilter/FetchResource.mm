/*!
  @file   FetchResource.mm
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Declaration of a C API to fetch system specific resources (iOS).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "FetchResource.h"

#import <Foundation/Foundation.h>
#include <Foundation/NSString.h>
#include <string>

static NSString *utf8(const std::string & name) { return [NSString stringWithUTF8String:name.c_str()]; }
static const char *cstring(NSString * name) { return [name cStringUsingEncoding:NSUTF8StringEncoding]; }
static const char *getPath(const std::string & resource)
{
    int dot = resource.rfind(".");
    auto name = resource.substr(0, dot), ext = resource.substr(dot+1, resource.size()-dot);
    NSString* path = [[NSBundle mainBundle] pathForResource:utf8(name) ofType:utf8(ext)];
    return cstring(path);
}

static std::string getPathAsString(const std::string &resource)
{
    std::string name = getPath(resource);
    return name;
}

static NSString *getDocumentsDir()
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    return [paths objectAtIndex:0];
}

std::string getResourcePath(const std::string &resource)
{
    return getPathAsString(resource);
}
