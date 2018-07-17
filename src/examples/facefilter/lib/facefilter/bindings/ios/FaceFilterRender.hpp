#ifndef __facefilter_bindings_ios_FaceFilterRender_hpp__
#define __facefilter_bindings_ios_FaceFilterRender_hpp__

#include <facefilter/renderer/facefilter_renderer.h>

#include <stdint.h>

#ifdef __cplusplus
FACEFILTER_EXTERN_C_BEGIN
#endif

void FaceFilterRender_loadAsset(const char* key, const char* url);
void FaceFilterRender_drawFrame(uint32_t texture);
void FaceFilterRender_surfaceChanged(int width, int height);
void FaceFilterRender_surfaceCreated(int width, int height);
void FaceFilterRender_cameraCreated(int width, int height, int rotation, float focalLength);
int FaceFilterRender_allocTexture(int width, int height);
void FaceFilterRender_registerCallback(void* classPtr, void (*callback)(void*));
void FaceFilterRender_setGLContext(void* context);

#ifdef __cplusplus
FACEFILTER_EXTERN_C_END
#endif

#endif // __facefilter_bindings_ios_FaceFilterRender_hpp__
