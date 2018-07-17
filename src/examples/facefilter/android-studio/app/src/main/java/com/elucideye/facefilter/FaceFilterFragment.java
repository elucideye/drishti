/*
  FaceFilterFragment.java
  
  Implementation of the main facefilter application fragment.

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

import android.app.Activity;
import android.graphics.Color;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.hardware.SensorManager;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.RelativeLayout;
import android.widget.Space;

public class FaceFilterFragment extends Fragment
    implements ActivityCompat.OnRequestPermissionsResultCallback
{
    private static String TAG = "ElucideyeFaceFilterFragment";
    private FaceFilterCameraManager mCameraManager = null;

    /**
    * An {@link FaceFilterGLSurfaceView} for camera preview.
    */
    private FaceFilterGLSurfaceView mGLSurfaceView;

    public static FaceFilterFragment newInstance(FaceFilterCameraManager cameraManager)
    {
        FaceFilterFragment fragment = new FaceFilterFragment();
        fragment.mCameraManager = cameraManager;
        return fragment;
    }

    @Override
    public View onCreateView(LayoutInflater inflater,
        ViewGroup container,
        Bundle savedInstanceState)
    {
        boolean attachToRoot = false;
        return inflater.inflate(R.layout.fragment, container, attachToRoot);
    }

    @Override
    public void onViewCreated(final View view, Bundle savedInstanceState)
    {
        mGLSurfaceView = view.findViewById(R.id.texture);
        mGLSurfaceView.onViewCreated(mCameraManager, this);
    }

    @Override
    public void onResume()
    {
        Log.d(TAG, "onResume");
        super.onResume();
        mGLSurfaceView.onResume();
        mCameraManager.onResume();
    }

    @Override
    public void onPause()
    {
        Log.d(TAG, "onPause");
        mCameraManager.onPause();
        mGLSurfaceView.onPause();
        super.onPause();
    }

    static { System.loadLibrary("facefilter_bindings"); }
}
