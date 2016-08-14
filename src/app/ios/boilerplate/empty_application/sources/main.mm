// Copyright (c) 2013-2014, Ruslan Baratov
// All rights reserved.

#import "AppDelegate.hpp"

int main(int argc, char *argv[]) {
  @autoreleasepool {
    NSString *principalClassName = nullptr; // UIApplication is assumed
    return UIApplicationMain(
        argc, argv, principalClassName, NSStringFromClass([AppDelegate class])
    );
  }
}
