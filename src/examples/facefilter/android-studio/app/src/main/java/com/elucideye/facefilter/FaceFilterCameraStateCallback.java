/*
  FaceFilterCameraStateCallback.java

  Implementation of the camera state callback for the facefilter session.

  Copyright 2017-2018 Elucideye, Inc. All rights reserved. [All modifications]

  This file is released under the 3 Clause BSD License. [All modifications]

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

import android.hardware.camera2.CameraDevice;
import android.util.Log;

import java.util.concurrent.Semaphore;

// All methods except constructor called from "camera thread"
public class FaceFilterCameraStateCallback extends CameraDevice.StateCallback
{
    private static String TAG = "ElucideyeFaceFilterCameraStateCallback";
    private FaceFilterCameraManager mCameraManager;

    FaceFilterCameraStateCallback(FaceFilterCameraManager cameraManager)
    {
        mCameraManager = cameraManager;
    }

    @Override
    public void onOpened(CameraDevice cameraDevice)
    {
        Log.d(TAG, "onOpened");
        mCameraManager.onCameraOpenResult(cameraDevice);
    }

    @Override
    public void onDisconnected(CameraDevice cameraDevice)
    {
        Log.d(TAG, "onDisconnected");
        cameraDevice.close();
        mCameraManager.onCameraOpenResult(null);
    }

    @Override
    public void onError(CameraDevice cameraDevice, int error)
    {
        Log.e(TAG, "onError: " + error);
        cameraDevice.close();
        mCameraManager.onCameraOpenResult(null);
    }
}
