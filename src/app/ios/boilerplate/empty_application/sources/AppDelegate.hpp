// Copyright (c) 2013-2014, Ruslan Baratov
// All rights reserved.

#import <UIKit/UIApplication.h> // UIApplicationDelegate
#import <UIKit/UIResponder.h> // UIResponder

@class UIWindow;
@class NSManagedObjectContext;
@class NSManagedObjectModel;
@class NSPersistentStoreCoordinator;

@interface AppDelegate : UIResponder <UIApplicationDelegate>

    @property (strong, nonatomic) UIWindow *window;

@property (readonly, strong, nonatomic)
NSManagedObjectContext *managedObjectContext;
@property (readonly, strong, nonatomic)
NSManagedObjectModel *managedObjectModel;
@property (readonly, strong, nonatomic)
NSPersistentStoreCoordinator *persistentStoreCoordinator;

- (void)saveContext;
- (NSURL *)applicationDocumentsDirectory;

@end
