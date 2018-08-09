/*
  FaceFilterRenderer.java
  
  Implementation of the facefilter rendering routines

  Copyright 2017-2018 Elucideye, Inc. All rights reserved. [All modifications]
  
  This file is released under the 3 Clause BSD License. [All modifications]

  Reference: http://maninara.blogspot.com/2015/03/render-camera-preview-using-opengl-es.html
*/
package com.elucideye.facefilter;

import android.opengl.GLSurfaceView;
import android.graphics.SurfaceTexture;

import android.content.res.AssetManager;

import android.opengl.GLES20;
import android.graphics.Point;

import android.opengl.GLES11Ext;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import android.util.Log;
import android.util.Size;
import android.view.Surface;

// We will receive GLSurfaceView.Renderer events for the underlying Surface of
// FaceFilterGLSurfaceView (GLSurfaceView) object to manage our own Surface
// and SurfaceTexture mSTexture. Original Surface will not be used directly.
public class FaceFilterRenderer
    implements GLSurfaceView.Renderer,
               SurfaceTexture.OnFrameAvailableListener
{
    private static String TAG = "ElucideyeFaceFilterRenderer";

    private int[] hTex;

    private SurfaceTexture mSTexture;

    private boolean mGLInit = false;
    private boolean mUpdateST = false;

    private Size mPreviewSize;
    private int mCameraOrientation;
    private float mFocalLengthInPixels;

    private FaceFilterGLSurfaceView mView;
    private FaceFilterCameraManager mCameraManager;
    private FaceFilterFragment mFragment;

    FaceFilterRenderer(FaceFilterGLSurfaceView view,
        FaceFilterCameraManager cameraManager,
        FaceFilterFragment fragment)
    {
        mView = view;
        mCameraManager = cameraManager;
        mPreviewSize = mCameraManager.calcPreviewSize();
        mCameraOrientation = mCameraManager.getCameraOrientation();
        mFocalLengthInPixels = mCameraManager.getFocalLengthInPixels();
        mFragment = fragment;

        // JNI
        setAssetManager(mFragment.getActivity().getAssets());

        loadAsset("drishti_assets", "drishti_assets.json");
    }

    // Scheduled from FaceFilterGLSurfaceView.onPause(). Executed from "render
    // thread". Thread: "render thread"
    public void onPause()
    {
        Log.d(TAG, "onPause");
        mGLInit = false;
        destroy(); // JNI
        mView.jniContextDestroyed();
    }

    // GLSurfaceView.Renderer
    // Thread: "render thread"
    // Note: this method can be called without previous requestRender
    @Override
    public void onDrawFrame(GL10 unused)
    {
        Log.d(TAG, "onDrawFrame");
        if (!mGLInit)
        {
            return;
        }

        boolean startDrawing = false;

        synchronized (this)
        {
            if (mUpdateST)
            {
                mSTexture.updateTexImage();
                mUpdateST = false;
                startDrawing = true;
            }
        }

        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT);

        if (startDrawing)
        {
            drawFrame(hTex[0]); // JNI
        }
    }

    // GLSurfaceView.Renderer
    // Thread: "render thread"
    @Override
    public void onSurfaceChanged(GL10 unused, int width, int height)
    {
        Log.d(TAG, "onSurfaceChanged");
        surfaceChanged(width, height); // JNI
    }

    // GLSurfaceView.Renderer
    // Thread: "render thread"
    @Override
    public void onSurfaceCreated(GL10 unused, EGLConfig config)
    {
        Log.d(TAG, "onSurfaceCreated");
        initTex();

        mSTexture = new SurfaceTexture(hTex[0]);
        mSTexture.setOnFrameAvailableListener(this);
        mSTexture.setDefaultBufferSize(mPreviewSize.getWidth(),
            mPreviewSize.getHeight());

        Surface cameraOutputSurface = new Surface(mSTexture);

        GLES20.glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        Point ss = new Point();
        mView.getDisplay().getRealSize(ss);

        // JNI
        surfaceCreated(ss.x,
            ss.y,
            mPreviewSize.getWidth(),
            mPreviewSize.getHeight(),
            mCameraOrientation,
            mFocalLengthInPixels);

        mGLInit = true;

        mCameraManager.onSurfaceCreated(cameraOutputSurface);
    }

    // SurfaceTexture.OnFrameAvailableListener
    // Thread: "UI thread"
    @Override
    public void onFrameAvailable(SurfaceTexture st)
    {
        Log.d(TAG, "onFrameAvailable");
        if (st != mSTexture)
        {
            Log.e(TAG, "onFrameAvailable received for the wrong SurfaceTexture");
        }

        synchronized (this) { mUpdateST = true; }

        mView.requestRender(); // Can be called from any thread
    }

    private void initTex()
    {
        hTex = new int[1];
        GLES20.glGenTextures(1, hTex, 0);
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, hTex[0]);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,
            GLES20.GL_TEXTURE_WRAP_S,
            GLES20.GL_CLAMP_TO_EDGE);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,
            GLES20.GL_TEXTURE_WRAP_T,
            GLES20.GL_CLAMP_TO_EDGE);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,
            GLES20.GL_TEXTURE_MIN_FILTER,
            GLES20.GL_NEAREST);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,
            GLES20.GL_TEXTURE_MAG_FILTER,
            GLES20.GL_NEAREST);
    }

    // Native C++ methods
    private native void setAssetManager(AssetManager assetManager);

    private native void loadAsset(String name, String path);

    private native void surfaceCreated(int displayWidth,
        int displayHeight,
        int cameraWidth,
        int cameraHeight,
        int cameraOrientation,
        float cameraFocalLength);
    private native void surfaceChanged(int width, int height);
    private native void drawFrame(int texture);
    private native void destroy();

    private native int allocTexture(int width, int height);
}
