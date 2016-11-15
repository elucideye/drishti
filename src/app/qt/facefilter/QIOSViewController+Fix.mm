#import "QIOSViewController+Fix.h"

#if QIOSVIEWCONTROLLER_FIX

@implementation UIViewController (Override)
- (void)setStatusBarHidden:(BOOL)hidden withAnimation:(BOOL)animation
{
    
}

- (void)setPrefersStatusBarHidden:(BOOL)hidden
{
    
}

- (void)setPreferredStatusBarUpdateAnimation:(UIStatusBarAnimation)animate
{

}

- (void)setPreferredStatusBarStyle:(UIStatusBarStyle)style
{

}
@end

#endif // QIOVIEWCONTROLLER_FIX
