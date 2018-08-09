/*
  FaceFilterGLSurfaceView.java
  
  Implementation of the facefilter GLSurfaceView

  Copyright 2017-2018 Elucideye, Inc. All rights reserved. [All modifications]
  
  This file is released under the 3 Clause BSD License. [All modifications]

  Reference: http://maninara.blogspot.com/2015/03/render-camera-preview-using-opengl-es.html

*/
package com.elucideye.facefilter;

import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.util.Log;
import android.view.SurfaceHolder;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class FaceFilterGLSurfaceView extends GLSurfaceView
{
    private static String TAG = "FaceFilterGLSurfaceView";

    FaceFilterRenderer mRenderer;

    private boolean mContextDestroyed;
    final Lock lock = new ReentrantLock();
    final Condition callbackFromRender = lock.newCondition();

    public FaceFilterGLSurfaceView(Context context) { this(context, null); }

    public FaceFilterGLSurfaceView(Context context, AttributeSet attrs)
    {
        super(context, attrs);
    }

    // Called from FaceFilterFragment.onViewCreated()
    public void onViewCreated(FaceFilterCameraManager cameraManager,
        FaceFilterFragment fragment)
    {
        if (mRenderer == null)
        {
            mRenderer = new FaceFilterRenderer(this, cameraManager, fragment);
        }

        setEGLContextClientVersion(3);

        // Register onSurfaceCreated listener
        // (note: setRenderer can be called only once in the life-cycle of the
        // GLSurfaceView)
        setRenderer(mRenderer);

        setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
    }

    // Called from FaceFilterFragment.onPause()
    @Override
    public void onPause()
    {
        mContextDestroyed = false;

        // We have to notify the "render thread" about the coming GL context
        // destruction event.
        queueEvent(new Runnable() {
            @Override
            public void run()
            {
                mRenderer.onPause();
            }
        });

        // Now we have to wait for the "render thread" to respond, otherwise we
        // can have a situation when the Java GL context is destroyed but the JNI
        // GL context related items not.
        lock.lock();
        try
        {
            while (!mContextDestroyed)
            {
                callbackFromRender.await();
            }
        }
        catch (InterruptedException e)
        {
            Log.e(TAG, "Await: InterruptedException." + e);
        }
        finally
        {
            lock.unlock();
        }

        // Confirmation received. JNI GL items are destroyed, so now we can destroy
        // the Java GL context.
        super.onPause();
    }

    // Called from FaceFilterRender.onPause
    // Thread: "render thread"
    public void jniContextDestroyed()
    {
        lock.lock();
        try
        {
            if (mContextDestroyed)
            {
                Log.e(TAG, "Already destroyed");
            }
            mContextDestroyed = true;
            callbackFromRender.signal();
        }
        finally
        {
            lock.unlock();
        }
    }
}
