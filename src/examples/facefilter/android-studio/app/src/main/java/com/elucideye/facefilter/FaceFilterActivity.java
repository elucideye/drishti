/*
  FaceFilterActivity.java
  
  Android Activity for the facefilter application

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

import android.os.Bundle;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;

// Activity will work with 3 threads:
// 1) UI thread: It's a main thread, all UI events here, e.g. onPause/onResume
// 2) Render thread: Created automatically by GLSurfaceView, see class
// FaceFilterRenderer 3) Camera thread: Thread to receive camera related events,
// see FaceFilterCamera*Callback classes
public class FaceFilterActivity extends AppCompatActivity
    implements ActivityCompat.OnRequestPermissionsResultCallback
{
    public FaceFilterCameraManager cameraManager;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.camera);

        if (cameraManager == null)
        {
            cameraManager = new FaceFilterCameraManager(this);
        }

        cameraManager.checkPermissions();

        FaceFilterFragment fragment = FaceFilterFragment.newInstance(cameraManager);
        getSupportFragmentManager()
            .beginTransaction()
            .replace(R.id.container, fragment)
            .commit();
    }

    // ActivityCompat.OnRequestPermissionsResultCallback
    @Override
    public void onRequestPermissionsResult(int requestCode,
        @NonNull String[] permissions,
        @NonNull int[] grantResults)
    {
        cameraManager.onRequestPermissionsResult(
            requestCode, permissions, grantResults);
    }
}
