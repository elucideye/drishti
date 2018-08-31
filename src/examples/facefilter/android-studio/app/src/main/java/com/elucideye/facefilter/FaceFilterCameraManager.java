/*
  FaceFilterCameraManager.java
  
  Implementation of main Android Camera 2 manager for the facefilter application.

  Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  
  This file is released under the 3 Clause BSD License.
  
*/

package com.elucideye.facefilter;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Camera;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.os.Handler;
import android.os.HandlerThread;
import android.support.annotation.NonNull;
import android.support.v4.content.ContextCompat;
import android.util.Log;
import android.hardware.camera2.CameraCharacteristics;
import android.util.Size;
import android.util.SizeF;
import android.view.Surface;

import java.util.Arrays;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;

// Method of this class can be called from different threads.
// All public methods guarded with mCameraLock.
public class FaceFilterCameraManager
{
    static private int REQUEST_CAMERA_PERMISSION = 1;
    static private String TAG = "FaceFilterCameraManager";

    private FaceFilterActivity mActivity;

    private boolean permissionsGranded = false;
    private boolean stateOnResume = false;
    private boolean surfaceCreated = false;
    private boolean cameraIsWorking = false;

    private HandlerThread mCameraBackgroundThread;
    private Handler mCameraBackgroundHandler;

    private Semaphore mCameraLock = new Semaphore(1);

    private CameraCaptureSession mCaptureSession;
    private CameraDevice mCameraDevice;

    private String mCameraID;
    private int mCameraOrientation;
    private float mFocalLengthInPixels;

    private CameraManager mCameraManager;
    private Surface mCameraOutputSurface = null;

    private FaceFilterCameraStateCallback mStateCallback;

    FaceFilterCameraManager(FaceFilterActivity activity)
    {
        mActivity = activity;
        mCameraManager = (CameraManager)activity.getSystemService(Context.CAMERA_SERVICE);
    }

    // Called from FaceFilterFragment.onResume()
    // Thread: "UI thread"
    public void onResume()
    {
        try
        {
            mCameraLock.acquire();

            if (stateOnResume)
            {
                Log.e(TAG, "Invalid onResume enter");
            }
            stateOnResume = true;
            analyzeState();
        }
        catch (InterruptedException e)
        {
            throw new RuntimeException("onResume", e);
        }
        finally
        {
            mCameraLock.release();
        }
    }

    // Called from FaceFilterFragment.onPause()
    // Thread: "UI thread"
    public void onPause()
    {
        try
        {
            mCameraLock.acquire();
            if (!stateOnResume)
            {
                Log.e(TAG, "Invalid onPause enter");
            }
            stateOnResume = false;
            surfaceCreated = false;
            analyzeState();
        }
        catch (InterruptedException e)
        {
            throw new RuntimeException("onPause", e);
        }
        finally
        {
            mCameraLock.release();
        }
    }

    // Called from FaceFilterRenderer.onSurfaceCreated()
    // Thread: "render thread"
    public void onSurfaceCreated(Surface cameraOutputSurface)
    {
        try
        {
            mCameraLock.acquire();
            if (surfaceCreated)
            {
                Log.e(TAG, "Invalid onSurfaceCreated enter");
            }

            surfaceCreated = true;
            mCameraOutputSurface = cameraOutputSurface;
            analyzeState();
        }
        catch (InterruptedException e)
        {
            throw new RuntimeException("onPause", e);
        }
        finally
        {
            mCameraLock.release();
        }
    }

    // Called from FaceFilterActivity.onCreate()
    // Thread: "UI thread"
    public void checkPermissions()
    {
        try
        {
            // Probably lock not needed here, just for unification.
            mCameraLock.acquire();

            int p = ContextCompat.checkSelfPermission(
                mActivity, Manifest.permission.WRITE_CALENDAR);
            if (p == PackageManager.PERMISSION_GRANTED)
            {
                permissionsGranded = true;
            }
            else
            {
                // Permission is not granted
                mActivity.requestPermissions(
                    new String[] { Manifest.permission.CAMERA },
                    REQUEST_CAMERA_PERMISSION);
            }
            analyzeState();
        }
        catch (InterruptedException e)
        {
            throw new RuntimeException("onPause", e);
        }
        finally
        {
            mCameraLock.release();
        }
    }

    // Called from FaceFilterActivity.onRequestPermissionsResult()
    // Thread: "UI thread"
    public void onRequestPermissionsResult(int requestCode,
        @NonNull String[] permissions,
        @NonNull int[] grantResults)
    {
        try
        {
            mCameraLock.acquire();

            if (requestCode != REQUEST_CAMERA_PERMISSION)
            {
                Log.e(TAG, "Permission result not for camera");
                return;
            }
            if (grantResults.length != 1)
            {
                Log.e(TAG, "Unexpected permission result length");
                return;
            }
            if (grantResults[0] != PackageManager.PERMISSION_GRANTED)
            {
                mActivity.finish();
                return;
            }

            permissionsGranded = true;
            analyzeState();
        }
        catch (InterruptedException e)
        {
            throw new RuntimeException("onPause", e);
        }
        finally
        {
            mCameraLock.release();
        }
    }

    private void analyzeState()
    {
        boolean newCameraIsWorking = permissionsGranded && stateOnResume && surfaceCreated;

        if (cameraIsWorking)
        {
            if (newCameraIsWorking)
            {
                // We got camera working and new state is "working" too.
                // Nothing changed, just continue.
                return;
            }

            // Currently camera is working, but new state is "not working".
            closeCamera();
            cameraIsWorking = false;
            return;
        }

        // Camera is not working here.

        if (newCameraIsWorking)
        {
            // New state is "working", starting camera.
            openCamera();
            cameraIsWorking = true;
            return;
        }
    }

    private void closeCamera()
    {
        if (null != mCaptureSession)
        {
            mCaptureSession.close();
            mCaptureSession = null;
        }
        if (null != mCameraDevice)
        {
            mCameraDevice.close();
            mCameraDevice = null;
        }
        stopCameraBackgroundThread();
    }

    private void openCamera()
    {
        Log.d(TAG, "openCamera");

        startCameraBackgroundThread();

        if (mStateCallback == null)
        {
            mStateCallback = new FaceFilterCameraStateCallback(this);
        }

        try
        {
            mCameraManager.openCamera(
                mCameraID, mStateCallback, mCameraBackgroundHandler);
        }
        catch (CameraAccessException e)
        {
            Log.e(TAG, "Camera open error: " + e);
        }
        catch (SecurityException e)
        {
            Log.e(TAG, "Camera open error: " + e);
        }
    }

    // Called from FaceFilterCameraSessionCallback.onConfigure
    // Thread: "camera thread"
    public void setCaptureSession(CameraCaptureSession session)
    {
        try
        {
            mCameraLock.acquire();
            if (null != mCaptureSession)
            {
                Log.e(TAG, "CameraCaptureSession already set");
            }
            if (null == session)
            {
                Log.e(TAG, "CameraCaptureSession is null");
                return;
            }
            if (null == mCameraDevice)
            {
                Log.e(TAG, "CameraDevice is null");
                return;
            }
            mCaptureSession = session;
        }
        catch (InterruptedException e)
        {
            throw new RuntimeException("Interrupted while trying to lock.", e);
        }
        finally
        {
            mCameraLock.release();
        }
    }

    // Called from FaceFilterCameraStateCallback methods
    // Thread: "camera thread"
    public void onCameraOpenResult(CameraDevice camera)
    {
        try
        {
            mCameraLock.acquire();

            mCameraDevice = camera;

            if (mCameraDevice == null)
            {
                return;
            }

            FaceFilterCameraSessionCallback callback = new FaceFilterCameraSessionCallback(
                this, mCameraDevice, mCameraBackgroundHandler, mCameraOutputSurface);

            mCameraDevice.createCaptureSession(
                Arrays.asList(mCameraOutputSurface), callback, null);
        }
        catch (InterruptedException e)
        {
            throw new RuntimeException("Interrupted while trying to lock.", e);
        }
        catch (CameraAccessException e)
        {
            Log.e(TAG, "createCaptureSession");
        }
        finally
        {
            mCameraLock.release();
        }
    }

    private void startCameraBackgroundThread()
    {
        Log.d(TAG, "startCameraBackgroundThread");
        mCameraBackgroundThread = new HandlerThread("CameraBackground");
        mCameraBackgroundThread.start();
        mCameraBackgroundHandler = new Handler(mCameraBackgroundThread.getLooper());
    }

    private void stopCameraBackgroundThread()
    {
        Log.d(TAG, "stopCameraBackgroundThread");
        mCameraBackgroundThread.quitSafely();
        try
        {
            mCameraBackgroundThread.join();
            mCameraBackgroundThread = null;
            mCameraBackgroundHandler = null;
        }
        catch (InterruptedException e)
        {
            Log.e(TAG, "stopCameraBackgroundThread");
        }
    }

    // Called from FaceFilterRenderer constructor
    // Thread: "UI thread"
    public Size calcPreviewSize()
    {
        Size previewSize = new Size(0, 0);
        try
        {
            for (String cameraID : mCameraManager.getCameraIdList())
            {
                CameraCharacteristics characteristics = mCameraManager.getCameraCharacteristics(cameraID);
                int facing = characteristics.get(CameraCharacteristics.LENS_FACING);
                if (facing != CameraCharacteristics.LENS_FACING_FRONT)
                {
                    continue;
                }

                int maxArea = 0;

                mCameraID = cameraID;
                mCameraOrientation = characteristics.get(CameraCharacteristics.SENSOR_ORIENTATION);

                StreamConfigurationMap map = characteristics.get(
                    CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                for (Size psize : map.getOutputSizes(SurfaceTexture.class))
                {
                    int area = psize.getHeight() * psize.getWidth();
                    if (area > maxArea)
                    {
                        maxArea = area;
                        previewSize = psize; // select max size
                    }
                }

                float focalLength = 1.0f;
                float focalLengths[] = characteristics.get(
                    CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS);
                switch (focalLengths.length)
                {
                    case 0:
                        Log.e(TAG, "Focal length not available");
                        break;
                    case 1:
                        focalLength = focalLengths[0];
                        break;
                    default:
                        Log.e(TAG, "Too many focal lengths");
                        focalLength = focalLengths[0];
                        break;
                }

                SizeF sensorSize = characteristics.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE);
                mFocalLengthInPixels = previewSize.getWidth() * focalLength / sensorSize.getWidth();

                break;
            }
        }
        catch (CameraAccessException e)
        {
            Log.e(TAG, "calcPreviewSize - Camera Access Exception");
        }
        catch (IllegalArgumentException e)
        {
            Log.e(TAG, "calcPreviewSize - Illegal Argument Exception");
        }
        catch (SecurityException e)
        {
            Log.e(TAG, "calcPreviewSize - Security Exception");
        }

        return previewSize;
    }

    // Called from FaceFilterRenderer constructor
    // Thread: "UI thread"
    public int getCameraOrientation() { return mCameraOrientation; }

    // Called from FaceFilterRenderer constructor
    // Thread: "UI thread"
    public float getFocalLengthInPixels() { return mFocalLengthInPixels; }
}
