#import <UIKit/UIKit.h>

#define QIOSVIEWCONTROLLER_FIX 1

#if QIOSVIEWCONTROLLER_FIX

@interface UIViewController (Override)
- (void)setStatusBarHidden:(BOOL)hidden withAnimation:(BOOL)animation;
- (void)setPrefersStatusBarHidden:(BOOL)hidden;
- (void)setPreferredStatusBarUpdateAnimation:(UIStatusBarAnimation)animate;
- (void)setPreferredStatusBarStyle:(UIStatusBarStyle)style;
@end

#endif
