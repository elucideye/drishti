macro(set_opencv_cmake_args_ios)
    set(OPENCV_CMAKE_ARGS 
      #### Repeat HUNTER ARGS ###
      BUILD_DOCS=OFF
      BUILD_TESTS=OFF
      BUILD_PERF_TESTS=OFF
      BUILD_opencv_apps=OFF
      BUILD_EXAMPLES=OFF
      ENABLE_NEON=ON 
      BUILD_ANDROID_SERVICE=OFF
      BUILD_ANDROID_EXAMPLES=OFF
      BUILD_ZLIB=OFF ## HUNTER
      BUILD_TIFF=OFF ## HUNTER
      BUILD_PNG=OFF  ## HUNTER
      ANDROID_EXAMPLES_WITH_LIBS=OFF    # "Build binaries of Android examples with native libraries" 

      ### Custom ARGS ###
      WITH_PNG=ON             # "Include PNG support"                          

      WITH_PTHREADS_PF=OFF     # "Use pthreads-based parallel_for"
      WITH_TBB=OFF            # "Include Intel TBB support"
      WITH_TIFF=OFF           # "Include TIFF support"                        
      WITH_JASPER=OFF         # "Include JPEG2K support"                      
      WITH_JPEG=OFF           # "Include JPEG support"                        
      WITH_1394=OFF           # "Include IEEE1394 support"          
      WITH_AVFOUNDATION=OFF   # "Use AVFoundation for Video I/O"             
      WITH_CARBON=OFF         # "Use Carbon for UI instead of Cocoa"        
      WITH_VTK=OFF            # "Include VTK library support (and build opencv_viz module eiher)" 
      WITH_CUDA=OFF           # "Include NVidia Cuda Runtime support"  
      WITH_CUFFT=OFF          # "Include NVidia Cuda Fast Fourier Transform (FFT) library support"          
      WITH_CUBLAS=OFF         # "Include NVidia Cuda Basic Linear Algebra Subprograms (BLAS) library support" 
      WITH_NVCUVID=OFF        # "Include NVidia Video Decoding library support"
      WITH_EIGEN=OFF          # "Include Eigen2/Eigen3 support" 
      WITH_VFW=OFF            # "Include Video for Windows support"         
      WITH_FFMPEG=OFF         # "Include FFMPEG support"                    
      WITH_GSTREAMER=OFF      # "Include Gstreamer support"                  
      WITH_GSTREAMER_0_10=OFF # "Enable Gstreamer 0.10 support (instead of 1.x)"
      WITH_GTK=OFF            # "Include GTK support"                         
      WITH_GTK_2_X=OFF        # "Use GTK version 2"                           
      WITH_IPP=OFF            # "Include Intel IPP support"                   
      WITH_WEBP=OFF           # "Include WebP support"                        
      WITH_OPENEXR=OFF        # "Include ILM support via OpenEXR"             
      WITH_OPENGL=OFF         # "Include OpenGL support"                      
      WITH_OPENNI=OFF         # "Include OpenNI support"                      
      WITH_OPENNI2=OFF        # "Include OpenNI2 support"                     
      WITH_PVAPI=OFF          # "Include Prosilica GigE support"              
      WITH_GIGEAPI=OFF        # "Include Smartek GigE support"                
      WITH_QT=OFF             # "Build with Qt Backend support"               
      WITH_WIN32UI=OFF        # "Build with Win32 UI Backend support"         
      WITH_QUICKTIME=OFF      # "Use QuickTime for Video I/O insted of QTKit" 
      WITH_TBB=OFF            # "Include Intel TBB support"                   
      WITH_OPENMP=OFF         # "Include OpenMP support"                      
      WITH_CSTRIPES=OFF       # "Include C= support"                          
      WITH_UNICAP=OFF         # "Include Unicap support (GPL)"                
      WITH_V4L=OFF            # "Include Video 4 Linux support"               
      WITH_LIBV4L=OFF         # "Use libv4l for Video 4 Linux support"        
      WITH_DSHOW=OFF          # "Build VideoIO with DirectShow support"       
      WITH_MSMF=OFF           # "Build VideoIO with Media Foundation support" 
      WITH_XIMEA=OFF          # "Include XIMEA cameras support"               
      WITH_XINE=OFF           # "Include Xine support (GPL)"                  
      WITH_CLP=OFF            # "Include Clp support (EPL)"                   
      WITH_OPENCL=OFF         # "Include OpenCL Runtime support"              
      WITH_OPENCL_SVM=OFF     # "Include OpenCL Shared Virtual Memory support" 
      WITH_OPENCLAMDFFT=OFF   # "Include AMD OpenCL FFT library support"      
      WITH_OPENCLAMDBLAS=OFF  # "Include AMD OpenCL BLAS library support"     
      WITH_DIRECTX=OFF        # "Include DirectX support"                     
      WITH_INTELPERC=OFF      # "Include Intel Perceptual Computing support"  
      WITH_IPP_A=OFF          # "Include Intel IPP_A support"                 
      WITH_GDAL=OFF           # "Include GDAL Support"                        
      WITH_GPHOTO2=OFF        # "Include gPhoto2 library support"             
    )
endmacro(set_opencv_cmake_args_ios)