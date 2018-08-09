/*
  FaceFilterCameraSessionCallback.java
  
  Implementation of camera processing callbacks for the facefilter session.

  Copyright 2017-2018 Elucideye, Inc. All rights reserved. [All modifications]
  
  This file is released under the 3 Clause BSD License.

  Lineage:

  - https://github.com/googlesamples/android-Camera2Basic

  ---

  Copyright 2017 The Android Open Source Project
  
  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
  
      http:www.apache.org/licenses/LICENSE-2.0
  
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/
package com.elucideye.facefilter;

import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CaptureRequest;
import android.os.Handler;
import android.util.Log;
import android.view.Surface;

// All methods except constructor called from "camera thread"
public class FaceFilterCameraSessionCallback
    extends CameraCaptureSession.StateCallback
{
    static private String TAG = "ElucideyeFaceFilterSessionCallback";

    private CaptureRequest.Builder mPreviewRequestBuilder;
    private Handler mBackgroundHandler;
    private FaceFilterCameraManager mCameraManager; // shared with "UI thread"

    FaceFilterCameraSessionCallback(FaceFilterCameraManager cameraManager,
        CameraDevice cameraDevice,
        Handler backgroundHandler,
        Surface cameraOutputSurface)
        throws CameraAccessException
    {
        mCameraManager = cameraManager;
        mBackgroundHandler = backgroundHandler;

        mPreviewRequestBuilder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
        mPreviewRequestBuilder.addTarget(cameraOutputSurface);
    }

    @Override
    public void onConfigured(CameraCaptureSession cameraCaptureSession)
    {
        Log.d(TAG, "onConfigured");

        mCameraManager.setCaptureSession(cameraCaptureSession);
        try
        {
            mPreviewRequestBuilder.set(
                CaptureRequest.CONTROL_AF_MODE,
                CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);

            mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AE_MODE,
                CaptureRequest.CONTROL_AE_MODE_ON_AUTO_FLASH);

            // After this call we will start receiving
            // SurfaceTexture.OnFrameAvailableListener -> onFrameAvailable events
            cameraCaptureSession.setRepeatingRequest(
                mPreviewRequestBuilder.build(), null, mBackgroundHandler);
        }
        catch (CameraAccessException e)
        {
            Log.e(TAG, "createCaptureSession");
        }
    }

    @Override
    public void onConfigureFailed(CameraCaptureSession cameraCaptureSession)
    {
        // TODO ???
    }
}
