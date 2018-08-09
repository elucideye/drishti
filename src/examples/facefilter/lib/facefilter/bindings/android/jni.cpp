/*! -*-c++-*-
  @file  jni.cpp
  @brief JNI bindings for facefilter rendering

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}
*/

#include <facefilter/bindings/android/Util.h>
#include <facefilter/renderer/Context.h>

#include <jni.h> // JNIEXPORT
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>

facefilter::Application g_app;

extern "C" JNIEXPORT void JNICALL
Java_com_elucideye_facefilter_FaceFilterRenderer_setAssetManager(JNIEnv* env, jobject thiz, jobject assetManager)
{
    LOGI("init w/ AAsetManager");

    g_app.setAssetManager(AAssetManager_fromJava(env, assetManager));
}

extern "C" JNIEXPORT void JNICALL
Java_com_elucideye_facefilter_FaceFilterRenderer_loadAsset(JNIEnv* env, jobject thiz, jstring name, jstring path)
{
    const char* path_ = env->GetStringUTFChars(path, nullptr);
    const char* name_ = env->GetStringUTFChars(name, nullptr);
    LOGI("setAssets w/ %s %s", name_, path_);

    g_app.loadAsset(name_, path_);
}

extern "C" JNIEXPORT void JNICALL
Java_com_elucideye_facefilter_FaceFilterRenderer_drawFrame(JNIEnv* env, jobject thiz, jint texture)
{
    LOGI("drawFrame %d", texture);

    g_app.drawFrame(texture);
}

extern "C" JNIEXPORT void JNICALL
Java_com_elucideye_facefilter_FaceFilterRenderer_surfaceChanged(JNIEnv* env, jobject thiz, jint width, jint height)
{
    LOGI("surfaceChanged %dx%d", width, height);

    g_app.initDisplay(width, height); // ?
}

extern "C" JNIEXPORT void JNICALL
Java_com_elucideye_facefilter_FaceFilterRenderer_surfaceCreated(
    JNIEnv* env,
    jobject thiz,
    jint displayWidth,
    jint displayHeight,
    jint cameraWidth,
    jint cameraHeight,
    jint cameraOrientation,
    jfloat cameraFocalLength)
{
    LOGI(
        "surfaceCreated display: %dx%d, camera: %dx%d[%d]{%f}",
        displayWidth,
        displayHeight,
        cameraWidth,
        cameraHeight,
        cameraOrientation,
        cameraFocalLength);

    g_app.initDisplay(displayWidth, displayHeight);
    g_app.initCamera(cameraWidth, cameraHeight, cameraOrientation, cameraFocalLength);
}

extern "C" JNIEXPORT void JNICALL
Java_com_elucideye_facefilter_FaceFilterRenderer_destroy(JNIEnv* env, jobject thiz)
{
    LOGI("cameraDestroyed");

    g_app.destroy();
}

extern "C" JNIEXPORT jint JNICALL
Java_com_elucideye_facefilter_FaceFilterRenderer_allocTexture(JNIEnv* env, jobject thiz, jint width, jint height)
{
    LOGI("allocTexture %dx%d", width, height);
    return 0;
}
