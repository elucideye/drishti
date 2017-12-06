
.. image:: https://user-images.githubusercontent.com/554720/33672534-72d8c01c-da78-11e7-8017-7c59dc282aac.jpg
   :width: 480pt

The ``drishti-hci`` application is similar in functionality to the ``drishti-face`` application, in that it will
compute full face models:

* face detection (typically Aggregated Channel Feature detection)
* face landmarks
* eye models

Where ``drishti-acf`` runs the above steps for batches of frames in parallel, ``drishti-hci`` processes a 
continuous video stream and uses a GPGPU camera pipeline for efficient processing.  This application 
configures an efficient face tracking pipeline that would be used on an actual mobile device.  It can
be used for tuning and offline analysis.  It leverages OpenGL ES 2.0 shaders targeting mobile platforms
(compatible syntax is used on desktop platforms) to perform basic frame manipulation (rotation/scaling) 
and for the ACF pyramid computation stage.  A small *drishti::videoio::VideoSourceCV* interface is used 
to read frames sequentially.  In order to process a video sequence you will need to provide a few parameters:

* focal length (in pixels)
* min detection distance (in meters)
* max detection distance (in meters)

An approximate focal length (from camera + lens specs) is sufficient in this case.  It is used only to 
configure the operative search volume in physical coordinates.  You do not need to perform an explicit 
calibration step for this.  You can obtain a reasonable estimate for the focal length using:

* lens focal length (mm)
* sensor size (mm)
* image size (pixels)

::

  focal_length_in_pixels = (image_width_in_pixels) * (focal_length_in_mm) / (CCD_width_in_mm)

**IMPORTANT**: You must build drishti with the ``DRISHTI_BUILD_TESTS=ON`` option to enable the OpenGL context that is required for the GPGPU processing that ``drishti-hci`` uses.  When that is provided to ``cmake`` (or ``polly ... --fwd DRISHTI_BUILD_TESTS=ON``) you will find the executable in the following installation tree path: ``_install/${TOOLCHAIN}/bin/drishti-hci``

Some trained models can be obtained from the `drishti-assets`_ repository.  You will find models for 
each stage of processing: face detection, face landmarks (to locate the eyes) and eye models.
In addition a fourth file is required, which contains mean face feature points from the full set of
detection data.  This is used to calibrate the face detector with the face landmark module -- it
provides the transformation mapping detection ROI's to the ROI's expected for landmark estimation.
These models can be stored in a single JSON file to simplify the command line arguments.  There are 
a few pre-populated JSON files that can be used to get started.  The basic format is shown below
`FaceDetectorFactory JSON format`_.

You can download initial models from here:

::

  cd ${HOME}/work && git clone https://github.com/elucideye/drishti-assets

On Apple platforms you can read MOV quicktime videos, such as those created from the iPhone camera 
application.  On other platforms you can process generic videos as a list of still images.  Additional 
`drishti::videoio::VideoSource` classes can be added as needed.

Create a list of stills video:

::

  find ${HOME}/video_stills/ -name "*.png" > /tmp/video_stills.txt
  
Process a video
  
::

  drishti-hci --factory=${HOME}/work/drishti_assets_big.json --input=${HOME}/video.mov --focal-length=4000 --min=0.5 --max=1.0 --output=/tmp/ --calibration=0.001 --scale=1.2 --window --resolution=0.5

Here is a quick breakdown of the parameters:

* ``--factory`` is used to specify the JSON file containing the models (see below)
* ``--input`` specifies the input video file
* ``--focal-length`` is the focal length in pixels
* ``--min`` is the min detection distance
* ``--max`` is the max detection distance
* ``--output`` is the directory for output files 
* ``--calibration`` is the ACF calibration term (can be used to adjust recall)
* ``--scale=1.2`` is used to further calibrate detection ROI's with landmark estimation ROI's
* ``--window`` is used to open a preview windows (not (ANDROID or IOS))
* ``--resolution`` reduce the preview window resolution as needed to fit the screen

The optional ``--swizzle`` argument can be used to specify permutations of ``RGBA`` (i.e., ``GRAB``, ``ARGB``, etc)
which controls the GGPU channels swizzling.  (AVFoundation videos from iOS typically required ``GRAB``.)
You can adjust ``--calibration=0.0001`` to adjust detection sensitivity (use a *small* negative value if you 
encounter false detections).  You can also use the ``--debug`` option to visualize the object detector scale search.  It will render a series of rectangles corresponding to the effective object detector window size resulting from search on the multi-resolution ACF pyramids that will look like this:

.. image:: https://user-images.githubusercontent.com/554720/33676055-ab48ec10-da82-11e7-87d1-689348182dfe.jpg
   :width: 480pt

You can use this to visualize the scale search resulting from your `--focal-length` + `--min=<min>` and `--max=<max>` parameters.

In a typical use case, once you instantiate a ``drishti::hci::FaceFinder`` and begin processing frames,
you will register a `drishti::hci::FaceMonitor` callback to get continuous per frame face models.
Note that these callbacks are blocking and should be handled efficiently to preserve real time behavior.

See the following sample `FaceMonitor_definition`_ for a sample ``drishti::hci::FaceMonitor`` definition 
and a correspoding code block for registering the callback: `FaceMonitor_registration`_. 

.. code:: c++

  // Simple FaceMonitor class to report face detection results over time.
  struct FaceMonitorLogger : public drishti::hci::FaceMonitor
  {
      FaceMonitorLogger(std::shared_ptr<spdlog::logger> &logger) : m_logger(logger) {}

      /**
       * A user defined virtual method callback that should report the number
       * of frames that should be captured from teh FIFO buffer based on the
       * reported face location.
       * @param faces a vector of faces for the current frame
       * @param timestmap the acquisition timestamp for the frame
       * @return a frame request for the last n frames with requested image formats
       */
      virtual Request request(const Faces& faces, const TimePoint& timeStamp)
      {
          // ~~~~~~~ YOU WOULD DO MOST OF YOUR ANALYSIS HERE ~~~~~~~
          cv::Point3f xyz = faces.size() ? (*faces.front().eyesCenter) : cv::Point3f();
          m_logger->info("SimpleFaceMonitor: Found {} faces {}", faces.size(), xyz);
          return {};
      }

      /**
       * A user defined virtual method callback that will be called with a
       * a populated vector of FaceImage objects for the last N frames, where
       * N is the number of frames requested in the preceding request callback.
       * @param frames A vector containing the last N consecutive FaceImage objects
       * @param isInitialized Return true if the FIFO buffer is fully initialized.
       */
      virtual void grab(const std::vector<FaceImage>& frames, bool isInitialized)
      {
          m_logger->info("SimpleFaceMonitor: Received {} frames", frames.size());
      }

      std::shared_ptr<spdlog::logger> m_logger;
  };

FaceDetectorFactory JSON format
===============================
 
::

  {
      "face_detector": "drishti_face_gray_80x80.cpb",
      "eye_model_regressor": "drishti_full_eye_model_big.cpb",
      "face_landmark_regressor": "drishti_full_face_model.cpb",
      "face_detector_mean": "drishti_face_gray_80x80_mean.json"
  }


This following command line was used to generate the image of the preview window shown at the top of the page:

::

  drishti-hci --factory=${HOME}/drishti-assets/drishti_assets_big.json --input=${HOME}/vimeo/Eyes_of_Hitchcock.mov --output=/tmp/ --scale=1.2 --window --swizzle=grab

.. _FaceMonitor_definition: https://github.com/elucideye/drishti/blob/0ab16cfea2b1046ab97c1c0d8d27cecb8c375bdb/src/app/hci/hci.cpp#L60-L96
.. _FaceMonitor_registration: https://github.com/elucideye/drishti/blob/0ab16cfea2b1046ab97c1c0d8d27cecb8c375bdb/src/app/hci/hci.cpp#L341-L344
.. _drishti-assets: https://github.com/elucideye/drishti-assets
