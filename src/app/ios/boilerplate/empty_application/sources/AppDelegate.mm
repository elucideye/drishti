// Copyright (c) 2013-2014, Ruslan Baratov
// All rights reserved.

#import "AppDelegate.hpp"

#include <boost/concept_check.hpp> // boost::ignore_unused_variable_warning

#import <CoreData/NSManagedObjectContext.h> // _managedObjectContext
#import <CoreData/NSManagedObjectModel.h> // _managedObjectModel
#import <CoreData/NSPersistentStoreCoordinator.h> // _persistentStoreCoordinator
#import <UIKit/UIScreen.h> // UIScreen
#import <UIKit/UIWindow.h> // UIWindow _window

@implementation AppDelegate

@synthesize window = _window;

@synthesize managedObjectContext = _managedObjectContext;
@synthesize managedObjectModel = _managedObjectModel;
@synthesize persistentStoreCoordinator = _persistentStoreCoordinator;

#pragma mark - UIApplicationDelegate implementation

- (BOOL)application:(UIApplication *)application
    didFinishLaunchingWithOptions:(NSDictionary *)launchOptions {
  boost::ignore_unused_variable_warning(application);
  boost::ignore_unused_variable_warning(launchOptions);
  self.window = [
      [UIWindow alloc] initWithFrame:[[UIScreen mainScreen] bounds]
  ];
  // Override point for customization after application launch.
  self.window.backgroundColor = [UIColor whiteColor];
  [self.window makeKeyAndVisible];
  return YES;
}

- (void)applicationWillResignActive:(UIApplication *)application {
  boost::ignore_unused_variable_warning(application);
  // Sent when the application is about to move from active to inactive
  // state. This can occur for certain types of temporary interruptions
  // (such as an incoming phone call or SMS message) or when the user
  // quits the application and it begins the transition to the background
  // state. Use this method to pause ongoing tasks, disable timers, and
  // throttle down OpenGL ES frame rates. Games should use this method
  // to pause the game.
}

- (void)applicationDidEnterBackground:(UIApplication *)application {
  boost::ignore_unused_variable_warning(application);
  // Use this method to release shared resources, save user data,
  // invalidate timers, and store enough application state information
  // to restore your application to its current state in case it is
  // terminated later.
  // If your application supports background execution,
  // this method is called instead of applicationWillTerminate:
  // when the user quits.
}

- (void)applicationWillEnterForeground:(UIApplication *)application {
  boost::ignore_unused_variable_warning(application);
  // Called as part of the transition from the background to the inactive
  // state; here you can undo many of the changes made on entering
  // the background.
}

- (void)applicationDidBecomeActive:(UIApplication *)application {
  boost::ignore_unused_variable_warning(application);
  // Restart any tasks that were paused (or not yet started) while
  // the application was inactive. If the application was previously
  // in the background, optionally refresh the user interface.
}

- (void)applicationWillTerminate:(UIApplication *)application {
  boost::ignore_unused_variable_warning(application);
  // Saves changes in the application's managed object context before
  // the application terminates.
  [self saveContext];
}

- (void)saveContext {
  NSError *error = nullptr;
  NSManagedObjectContext *managedObjectContext = self.managedObjectContext;
  if (managedObjectContext == nullptr) {
    return;
  }
  if (![managedObjectContext hasChanges]) {
    return;
  }
  if (![managedObjectContext save:&error]) {
    // Replace this implementation with code to handle the error appropriately.
    // abort() causes the application to generate a crash log and terminate.
    // You should not use this function in a shipping application,
    // although it may be useful during development.
    NSLog(@"Unresolved error %@, %@", error, [error userInfo]);
    abort();
  }
}

#pragma mark - Core Data stack

// Returns the managed object context for the application.
// If the context doesn't already exist, it is created and bound to the
// persistent store coordinator for the application.
- (NSManagedObjectContext *)managedObjectContext {
  if (_managedObjectContext != nullptr) {
    return _managedObjectContext;
  }

  NSPersistentStoreCoordinator *coordinator = [self persistentStoreCoordinator];
  if (coordinator != nullptr) {
    _managedObjectContext = [[NSManagedObjectContext alloc] init];
    [_managedObjectContext setPersistentStoreCoordinator:coordinator];
  }
  return _managedObjectContext;
}

// Returns the managed object model for the application.
// If the model doesn't already exist,
// it is created from the application's model.
- (NSManagedObjectModel *)managedObjectModel {
  if (_managedObjectModel != nullptr) {
    return _managedObjectModel;
  }
  NSString* resource = @"FIXME"; // FIXME
  NSURL *modelURL = [
      [NSBundle mainBundle] URLForResource:resource withExtension:@"momd"
  ];
  _managedObjectModel = [
      [NSManagedObjectModel alloc] initWithContentsOfURL:modelURL
  ];
  return _managedObjectModel;
}

// Returns the persistent store coordinator for the application.
// If the coordinator doesn't already exist,
// it is created and the application's store added to it.
- (NSPersistentStoreCoordinator *)persistentStoreCoordinator {
  if (_persistentStoreCoordinator != nullptr) {
    return _persistentStoreCoordinator;
  }

  NSString* comp = @"FIXME.sqlite"; // FIXME
  NSURL *storeURL = [
      [self applicationDocumentsDirectory] URLByAppendingPathComponent:comp
  ];

  NSError *error = nullptr;
  _persistentStoreCoordinator = [
      [NSPersistentStoreCoordinator alloc]
          initWithManagedObjectModel:[self managedObjectModel]
  ];
  const bool ok = [
      _persistentStoreCoordinator
          addPersistentStoreWithType:NSSQLiteStoreType
          configuration:nullptr
          URL:storeURL
          options:nullptr
          error:&error
  ];

  if (!ok) {
    // Replace this implementation with code to handle the error appropriately.
    //
    // abort() causes the application to generate a crash log and terminate.
    // You should not use this function in a shipping application,
    // although it may be useful during development.
    //
    // Typical reasons for an error here include:
    // * The persistent store is not accessible;
    // * The schema for the persistent store is incompatible with current
    // managed object model.
    // Check the error message to determine what the actual problem was.
    //
    //
    // If the persistent store is not accessible, there is typically something
    // wrong with the file path. Often, a file URL is pointing into the
    // application's resources directory instead of a writeable directory.
    //
    // If you encounter schema incompatibility errors during development,
    // you can reduce their frequency by:
    // * Simply deleting the existing store:
    // [[NSFileManager defaultManager] removeItemAtURL:storeURL error:nullptr]
    //
    // * Performing automatic lightweight migration by passing the following
    // dictionary as the options parameter:
    // @{NSMigratePersistentStoresAutomaticallyOption:@YES,
    //     NSInferMappingModelAutomaticallyOption:@YES}
    //
    // Lightweight migration will only work for a limited set of schema changes;
    // consult "Core Data Model Versioning and Data Migration Programming Guide"
    // for details.
    NSLog(@"Unresolved error %@, %@", error, [error userInfo]);
    abort();
  }

  return _persistentStoreCoordinator;
}

#pragma mark - Application's Documents directory

// Returns the URL to the application's Documents directory.
- (NSURL *)applicationDocumentsDirectory {
  NSFileManager* manager = [NSFileManager defaultManager];
  NSArray* urls =
      [manager URLsForDirectory:NSDocumentDirectory inDomains:NSUserDomainMask];
  return [urls lastObject];
}

@end
