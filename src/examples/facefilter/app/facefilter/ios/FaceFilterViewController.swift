/*!
 @file   FaceFilterViewController.swift
 @brief  View Controller for the facefilter application (swift)
   
 \copyright Copyright 2014-2018 Elucideye, Inc. All rights reserved. [All modifications]
 \license{This project is released under the 3 Clause BSD License.} [All modifications]

 View controller that handles camera, drawing, and touch events.

 ---------------

 Lineage:

 Version: 1.0

 Disclaimer: IMPORTANT:  This Apple software is supplied to you by Apple
 Inc. ("Apple") in consideration of your agreement to the following
 terms, and your use, installation, modification or redistribution of
 this Apple software constitutes acceptance of these terms.  If you do
 not agree with these terms, please do not use, install, modify or
 redistribute this Apple software.

 In consideration of your agreement to abide by the following terms, and
 subject to these terms, Apple grants you a personal, non-exclusive
 license, under Apple's copyrights in this original Apple software (the
 "Apple Software"), to use, reproduce, modify and redistribute the Apple
 Software, with or without modifications, in source and/or binary forms;
 provided that if you redistribute the Apple Software in its entirety and
 without modifications, you must retain this notice and the following
 text and disclaimers in all such redistributions of the Apple Software.
 Neither the name, trademarks, service marks or logos of Apple Inc. may
 be used to endorse or promote products derived from the Apple Software
 without specific prior written permission from Apple.  Except as
 expressly stated in this notice, no other rights or licenses, express or
 implied, are granted by Apple herein, including but not limited to any
 patent rights that may be infringed by your derivative works or by other
 works in which the Apple Software may be incorporated.

 The Apple Software is provided by Apple on an "AS IS" basis.  APPLE
 MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION
 THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE, REGARDING THE APPLE SOFTWARE OR ITS USE AND
 OPERATION ALONE OR IN COMBINATION WITH YOUR PRODUCTS.

 IN NO EVENT SHALL APPLE BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL
 OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE, REPRODUCTION,
 MODIFICATION AND/OR DISTRIBUTION OF THE APPLE SOFTWARE, HOWEVER CAUSED
 AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE),
 STRICT LIABILITY OR OTHERWISE, EVEN IF APPLE HAS BEEN ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 Copyright (C) 2013 Apple Inc. All Rights Reserved.

 Translated by OOPer in cooperation with shlab.jp, on 2016/9/5.
 */

import AVFoundation
import GLKit
import UIKit

import CoreVideo.CVOpenGLESTextureCache

// Uniform index.
private let UNIFORM_Y = 0
private let UNIFORM_UV = 1
private let NUM_UNIFORMS = 2
private var uniforms: [GLint] = [0, 0]

// Attribute index.
private let ATTRIB_VERTEX = 0
private let ATTRIB_TEXCOORD = 1
private let NUM_ATTRIBUTES = 2

struct ShaderCompileError: Error
{
    let compileLog: String
}

// http://www.perry.cz/clanky/swift.html
func bridge(_ obj: AnyObject) -> UnsafeMutableRawPointer
{
    return Unmanaged.passUnretained(obj).toOpaque()
}

// http://www.perry.cz/clanky/swift.html
func bridge<T: AnyObject>(_ ptr: UnsafeMutableRawPointer?) -> T
{
    return Unmanaged<T>.fromOpaque(ptr!).takeUnretainedValue()
}

class FaceFilterViewController: GLKViewController, AVCaptureVideoDataOutputSampleBufferDelegate
{
    private var _program: GLuint = 0

    private var _positionVBO: GLuint = 0
    private var _texcoordVBO: GLuint = 0
    private var _indexVBO: GLuint = 0

    private var _backingDimensions: GLSize = GLSize(width: 0, height: 0)
    private var _textureDimensions: GLSize = GLSize(width: 0, height: 0)
    private var _videoDimensions: CMVideoDimensions = CMVideoDimensions(width: 0, height: 0)

    private var _indexCount: Int = 0
    private var _indexSize: Int = 0
    private var _vertexSize: Int = 0
    private var _indices: UnsafeMutablePointer<GLushort>!
    private var _vertices: UnsafeMutablePointer<GLfloat>!
    private var _texCoords: UnsafeMutablePointer<GLfloat>!

    private var _context: EAGLContext!

    private var _eaglLayer: CAEAGLLayer!

    private var _lumaTexture: CVOpenGLESTexture?
    private var _chromaTexture: CVOpenGLESTexture?

    private var _sessionPreset: String = AVCaptureSession.Preset.vga640x480.rawValue

    private var _session: AVCaptureSession!
    private var _videoTextureCache: CVOpenGLESTextureCache?

    private var _cameraRotation: Int32 = 90
    private var _cameraFocalLength: Float = 0.0

    private func allocBuffers()
    {
        _indexCount = 4
        _indexSize = _indexCount * MemoryLayout<GLushort>.size
        _vertexSize = 2 * _indexCount * MemoryLayout<GLfloat>.size

        _indices = UnsafeMutablePointer<GLushort>.allocate(capacity: _indexCount)
        let indicesValues: [GLushort] = [0, 1, 2, 3]
        for (index, value) in indicesValues.enumerated()
        {
            _indices[index] = value
        }

        _vertices = UnsafeMutablePointer<GLfloat>.allocate(capacity: _indexCount * 2)
        let verticesValues: [GLfloat] = [-1, -1, +1, -1, -1, +1, +1, +1]
        for (index, value) in verticesValues.enumerated()
        {
            _vertices[index] = value
        }

        _texCoords = UnsafeMutablePointer<GLfloat>.allocate(capacity: _indexCount * 2)
        let texCoordsValues: [GLfloat] = [0, 0, 1, 0, 0, 1, 1, 1]
        for (index, value) in texCoordsValues.enumerated()
        {
            _texCoords[index] = value
        }
    }

    var glView: GLKView
    {
        return view as! GLKView
    }

    var eaglLayer: CAEAGLLayer
    {
        return view.layer as! CAEAGLLayer
    }

    @objc func initDraw()
    {
        print("test func in view contriller ")
        glView.bindDrawable()
    }
    
    override func viewDidAppear(_ animated: Bool)
    {
        super.viewDidAppear(animated)
        getBackingDimensions()
        FaceFilterRender_surfaceCreated(_backingDimensions.width, _backingDimensions.height)
    }

    func loadAssets()
    {
        guard let path = Bundle.main.path(forResource: "drishti_assets", ofType: "json") else
        {
            // TODO: consistent error handling (w/ notification) on failure
            print("Failed to locat required resource: drishti_assets.json")
            abort()
        }

        FaceFilterRender_loadAsset("drishti_assets", (path as NSString).utf8String)
    }

    // https://github.com/bradley/iOSSwiftOpenGLCamera/blob/master/iOSSwiftOpenGLCamera/OpenGLView.swift
    override func viewDidLoad()
    {
        super.viewDidLoad()
        
#if DRISHTI_OPENGL_ES3
        _context = EAGLContext(api: .openGLES3)
#else
        _context = EAGLContext(api: .openGLES2)
#endif

        if _context == nil
        {
            print("Failed to create ES context")
        }

        // CALayer's are, by default, non-opaque, which is 'bad for performance with OpenGL',
        //   so let's set our CAEAGLLayer layer to be opaque.
        eaglLayer.isOpaque = true

        glView.bindDrawable()
        glView.context = _context

        preferredFramesPerSecond = 60
        view.contentScaleFactor = UIScreen.main.scale

        _sessionPreset = AVCaptureSession.Preset.high.rawValue

        allocBuffers()
        setupBuffers()
        setupAVCapture() // before setupGL
        setupGL()

        let rawSelf = bridge(self)

        let simpleCallback: @convention(c) (UnsafeMutableRawPointer?) -> Void = {
            rawSelfFromCallback in
            // Extract pointer to `self` from void pointer:
            let me: FaceFilterViewController = bridge(rawSelfFromCallback)
            me.initDraw()
        }

        loadAssets()

        // void * classPtr, void(*callback)(void *)
        FaceFilterRender_registerCallback(rawSelf, simpleCallback)

        FaceFilterRender_cameraCreated(_videoDimensions.width, _videoDimensions.height, _cameraRotation, _cameraFocalLength)

        do
        {
            try setupOutputTexture(width: _videoDimensions.width, height: _videoDimensions.height)
        }
        catch
        {
            // TODO: Terminate
            print("Total failure")
            abort()
        }

        _session.startRunning() // start running
    }

    override func viewWillDisappear(_ animated: Bool)
    {
        super.viewWillDisappear(animated)

        tearDownAVCapture()
        tearDownGL()
        tearDownOutputTexture()

        if EAGLContext.current() === _context
        {
            EAGLContext.setCurrent(nil)
        }
    }

    override func didReceiveMemoryWarning()
    {
        super.didReceiveMemoryWarning()
        // Release any cached data, images, etc. that aren't in use.
    }

    // Camera image orientation on screen is fixed
    // with respect to the physical camera orientation.
    override var supportedInterfaceOrientations: UIInterfaceOrientationMask
    {
        return .portrait
    }

    override var shouldAutorotate: Bool
    {
        return true
    }

    private func cleanUpTextures()
    {
        _lumaTexture = nil
        _chromaTexture = nil

        // Periodic texture cache flush every frame
        CVOpenGLESTextureCacheFlush(_videoTextureCache!, 0)
        checkGLError()
    }

    private var _hasFrame = false

    func captureOutput(_: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from _: AVCaptureConnection)
    {
        print("Running on \(Thread.current) thread (captureOutput)")

        checkGLError()

        EAGLContext.setCurrent(_context)

        let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer)!

        checkGLError()

        guard let videoTextureCache = _videoTextureCache else
        {
            print("No video texture cache")
            return
        }

        checkGLError()

        cleanUpTextures()

        // CVOpenGLESTextureCacheCreateTextureFromImage will create GLES texture
        // optimally from CVImageBufferRef.

        checkGLError()
        
#if DRISHTI_OPENGL_ES3
        // OpenGL ES 3.0
        let luminanceKind = GL_LUMINANCE
        let chrominanceKind = GL_LUMINANCE_ALPHA
#else
        // OpenGL ES 2.0
        let luminanceKind = GL_RED_EXT
        let chrominanceKind = GL_RG_EXT
#endif

        // Y-plane
        glActiveTexture(GLenum(GL_TEXTURE4))
        checkGLError()
        var err = CVOpenGLESTextureCacheCreateTextureFromImage(
            kCFAllocatorDefault,
            videoTextureCache,
            pixelBuffer,
            nil,
            GLenum(GL_TEXTURE_2D),
            luminanceKind,
            GLsizei(_textureDimensions.width),
            GLsizei(_textureDimensions.height),
            GLenum(luminanceKind),
            GLenum(GL_UNSIGNED_BYTE),
            0,
            &_lumaTexture)
        if err != 0 {
            print("Error at CVOpenGLESTextureCacheCreateTextureFromImage \(err)")
        }

        checkGLError()
        glBindTexture(CVOpenGLESTextureGetTarget(_lumaTexture!), CVOpenGLESTextureGetName(_lumaTexture!))
        checkGLError()
        glTexParameterf(GLenum(GL_TEXTURE_2D), GLenum(GL_TEXTURE_WRAP_S), GLfloat(GL_CLAMP_TO_EDGE))
        checkGLError()
        glTexParameterf(GLenum(GL_TEXTURE_2D), GLenum(GL_TEXTURE_WRAP_T), GLfloat(GL_CLAMP_TO_EDGE))
        checkGLError()

        // UV-plane
        glActiveTexture(GLenum(GL_TEXTURE5))
        checkGLError()
        err = CVOpenGLESTextureCacheCreateTextureFromImage(
            kCFAllocatorDefault,
            videoTextureCache,
            pixelBuffer,
            nil,
            GLenum(GL_TEXTURE_2D),
            chrominanceKind,
            GLsizei(_textureDimensions.width / 2),
            GLsizei(_textureDimensions.height / 2),
            GLenum(chrominanceKind),
            GLenum(GL_UNSIGNED_BYTE),
            1,
            &_chromaTexture)
        if err != 0 {
            print("Error at CVOpenGLESTextureCacheCreateTextureFromImage \(err)")
        }

        checkGLError()
        glBindTexture(CVOpenGLESTextureGetTarget(_chromaTexture!), CVOpenGLESTextureGetName(_chromaTexture!))
        checkGLError()
        glTexParameterf(GLenum(GL_TEXTURE_2D), GLenum(GL_TEXTURE_WRAP_S), GLfloat(GL_CLAMP_TO_EDGE))
        checkGLError()
        glTexParameterf(GLenum(GL_TEXTURE_2D), GLenum(GL_TEXTURE_WRAP_T), GLfloat(GL_CLAMP_TO_EDGE))
        checkGLError()

        renderToTexture()

        _hasFrame = true
    }

    private func getBackingDimensions()
    {
        _backingDimensions = GLSize(width: GLint(glView.drawableWidth), height: GLint(glView.drawableHeight))
    }

    private var _textureOutput: GLuint = 0
    private var _frameBuffer: GLuint = 0
    private var _colorRenderbuffer: GLuint = 0

    private func tearDownOutputTexture()
    {
        glDeleteBuffers(1, &_frameBuffer)
        checkGLError()
        glDeleteTextures(1, &_textureOutput)
        checkGLError()
    }

    private func setupOutputTexture(width: GLint, height: GLint) throws
    {
        _textureOutput = generateTexture(
            minFilter: GL_LINEAR,
            magFilter: GL_LINEAR,
            wrapS: GL_CLAMP_TO_EDGE,
            wrapT: GL_CLAMP_TO_EDGE)
        try _frameBuffer = generateFramebufferForTexture(
            _textureOutput,
            width: width,
            height: height,
            internalFormat: GL_RGBA,
            format: GL_RGBA,
            type: GL_UNSIGNED_BYTE,
            stencil: false).0
    }

    private func renderToTexture()
    {
        // Enable the shader:
        checkGLError()

        glUseProgram(_program)
        checkGLError()
        glViewport(0, 0, GLint(_textureDimensions.width), GLint(_textureDimensions.height))
        checkGLError()

        // L and UV will be bound to 4 and 5 respectively:
        glActiveTexture(GLenum(GL_TEXTURE4))
        glUniform1i(uniforms[UNIFORM_Y], 4)
        checkGLError()

        glActiveTexture(GLenum(GL_TEXTURE5))
        glUniform1i(uniforms[UNIFORM_UV], 5)
        checkGLError()

        // Bind an output FBO:
        glBindFramebuffer(GLenum(GL_FRAMEBUFFER), _frameBuffer)
        checkGLError()

        // glClear(GLbitfield(GL_COLOR_BUFFER_BIT))
        // checkGLError()
        // glClearColor(1.0, 0.0, 1.0, 1.0)
        // checkGLError()

        // Enable the vertex attribute array
        glBindBuffer(GLenum(GL_ARRAY_BUFFER), _positionVBO)
        glEnableVertexAttribArray(GLuint(ATTRIB_VERTEX))
        glVertexAttribPointer(GLuint(ATTRIB_VERTEX), 2, GLenum(GL_FLOAT), GLboolean(GL_FALSE), GLsizei(2 * MemoryLayout<GLfloat>.size), nil)
        checkGLError()

        glBindBuffer(GLenum(GL_ARRAY_BUFFER), _texcoordVBO)
        glEnableVertexAttribArray(GLuint(ATTRIB_TEXCOORD))
        glVertexAttribPointer(GLuint(ATTRIB_TEXCOORD), 2, GLenum(GL_FLOAT), GLboolean(GL_FALSE), GLsizei(2 * MemoryLayout<GLfloat>.size), nil)
        checkGLError()

        glBindBuffer(GLenum(GL_ELEMENT_ARRAY_BUFFER), _indexVBO)
        checkGLError()

        glDrawElements(GLenum(GL_TRIANGLE_STRIP), GLsizei(4), GLenum(GL_UNSIGNED_SHORT), nil)
        checkGLError()

        ///// DONE /////

        glDisableVertexAttribArray(GLuint(ATTRIB_TEXCOORD))
        checkGLError()
        glDisableVertexAttribArray(GLuint(ATTRIB_VERTEX))
        checkGLError()

        // glView.bindDrawable() // i.e, glBindFramebuffer(GLenum(GL_FRAMEBUFFER), 0)

        glBindBuffer(GLenum(GL_ARRAY_BUFFER), 0)
        checkGLError()
        glBindBuffer(GLenum(GL_ELEMENT_ARRAY_BUFFER), 0)
        checkGLError()
    }

    private func setupAVCapture()
    {
        // -- Create CVOpenGLESTextureCacheRef for optimal CVImageBufferRef to GLES texture conversion.
        let err = CVOpenGLESTextureCacheCreate(kCFAllocatorDefault, nil, _context, nil, &_videoTextureCache)
        guard err == 0 else
        {
            print("Error at CVOpenGLESTextureCacheCreate \(err)")
            return
        }

        // -- Setup Capture Session.
        _session = AVCaptureSession()
        _session.beginConfiguration()

        // -- Set preset session size.
        _session.canSetSessionPreset(AVCaptureSession.Preset(rawValue: _sessionPreset))

        // -- Creata a video device and input from that Device.  Add the input to the capture session.
        let videoDevice = AVCaptureDevice.devices().first(where: { ($0 as AnyObject).position == .front })!

        // -- Add the device to the session.
        let input: AVCaptureDeviceInput
        do
        {
            input = try AVCaptureDeviceInput(device: videoDevice)
        }
        catch
        {
            fatalError(error.localizedDescription)
        }

        _session.addInput(input)

        // -- Create the output for the capture session.
        let dataOutput = AVCaptureVideoDataOutput()
        dataOutput.alwaysDiscardsLateVideoFrames = true // Probably want to set this to NO when recording

        // -- Set to YUV420.
        dataOutput.videoSettings = [
            kCVPixelBufferPixelFormatTypeKey as NSString: NSNumber(value: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange),
        ] as [String: Any]

        // Set dispatch to be on the main thread so OpenGL can do things with the data
        dataOutput.setSampleBufferDelegate(self, queue: DispatchQueue.main)

        _session.addOutput(dataOutput)
        _session.commitConfiguration()

        let formatDescription = videoDevice.activeFormat.formatDescription
        _videoDimensions = CMVideoFormatDescriptionGetDimensions(formatDescription)
        _textureDimensions = GLSize(width: _videoDimensions.width, height: _videoDimensions.height)
        _cameraRotation = Int32(90)

        // https://stackoverflow.com/a/29597490
        let hfov = videoDevice.activeFormat.videoFieldOfView * Float.pi / 180.0
        _cameraFocalLength = Float(_videoDimensions.width) / 2.0 / tan(hfov / 2.0)

        print("w \(_textureDimensions.width) h \(_textureDimensions.height)")

        // Defer running until setup is complete:
        // _session?.startRunning()
    }

    private func tearDownAVCapture()
    {
        cleanUpTextures()

        _videoTextureCache = nil
    }

    func setupBuffers()
    {
        EAGLContext.setCurrent(_context)

        checkGLError()
        glGenBuffers(1, &_indexVBO)
        checkGLError()
        glBindBuffer(GLenum(GL_ELEMENT_ARRAY_BUFFER), _indexVBO)
        checkGLError()
        glBufferData(GLenum(GL_ELEMENT_ARRAY_BUFFER), _indexSize, UnsafePointer(_indices), GLenum(GL_STATIC_DRAW))
        checkGLError()

        glGenBuffers(1, &_positionVBO)
        checkGLError()
        glBindBuffer(GLenum(GL_ARRAY_BUFFER), _positionVBO)
        checkGLError()
        glBufferData(GLenum(GL_ARRAY_BUFFER), _vertexSize, UnsafePointer(_vertices), GLenum(GL_STATIC_DRAW))
        checkGLError()
        glEnableVertexAttribArray(GLuint(ATTRIB_VERTEX))
        checkGLError()
        glVertexAttribPointer(GLuint(ATTRIB_VERTEX), 2, GLenum(GL_FLOAT), GLboolean(GL_FALSE), GLsizei(2 * MemoryLayout<GLfloat>.size), nil)
        checkGLError()

        glGenBuffers(1, &_texcoordVBO)
        checkGLError()
        glBindBuffer(GLenum(GL_ARRAY_BUFFER), _texcoordVBO)
        checkGLError()
        glBufferData(GLenum(GL_ARRAY_BUFFER), _vertexSize, UnsafePointer(_texCoords), GLenum(GL_DYNAMIC_DRAW))
        checkGLError()
        glEnableVertexAttribArray(GLuint(ATTRIB_TEXCOORD))
        checkGLError()
        glVertexAttribPointer(GLuint(ATTRIB_TEXCOORD), 2, GLenum(GL_FLOAT), GLboolean(GL_FALSE), GLsizei(2 * MemoryLayout<GLfloat>.size), nil)
        checkGLError()
    }

    private func setupGL()
    {
        EAGLContext.setCurrent(_context)

        checkGLError()
        loadShaders()

        checkGLError()
        glUseProgram(_program)
        checkGLError()
        glUniform1i(uniforms[UNIFORM_Y], 4)
        checkGLError()
        glUniform1i(uniforms[UNIFORM_UV], 5)
        checkGLError()
    }

    private func tearDownGL()
    {
        EAGLContext.setCurrent(_context)

        glDeleteBuffers(1, &_positionVBO)
        checkGLError()
        glDeleteBuffers(1, &_texcoordVBO)
        checkGLError()
        glDeleteBuffers(1, &_indexVBO)
        checkGLError()

        if _program != 0 {
            glDeleteProgram(_program)
            checkGLError()
            _program = 0
        }

        tearDownOutputTexture()
    }

    // MARK: - GLKView and GLKViewController delegate methods

    func update()
    {
    }

    override func glkView(_: GLKView, drawIn _: CGRect)
    {
        EAGLContext.setCurrent(_context)

        if _hasFrame
        {
            // NSLog("Running on %@ thread", Thread.current)
            FaceFilterRender_drawFrame(_textureOutput)
        }
    }
 
    // MARK: - OpenGL ES 2 shader compilation

    @discardableResult
    func loadShaders() -> Bool
    {
        var vertShader: GLuint = 0, fragShader: GLuint = 0

        // Create shader program.
        checkGLError()
        _program = glCreateProgram()

        checkGLError()
        // Create and compile vertex shader.
        let vertShaderURL = Bundle.main.url(forResource: "Shader", withExtension: "vsh")!

        guard compileShader(&vertShader, type: GLenum(GL_VERTEX_SHADER), url: vertShaderURL) else
        {
            print("Failed to compile vertex shader")
            return false
        }

        // Create and compile fragment shader.
        let fragShaderURL = Bundle.main.url(forResource: "Shader", withExtension: "fsh")!

        guard compileShader(&fragShader, type: GLenum(GL_FRAGMENT_SHADER), url: fragShaderURL) else
        {
            print("Failed to compile fragment shader")
            return false
        }

        // Attach vertex shader to program.
        glAttachShader(_program, vertShader)

        // Attach fragment shader to program.
        glAttachShader(_program, fragShader)

        // Bind attribute locations.
        // This needs to be done prior to linking.
        glBindAttribLocation(_program, GLuint(ATTRIB_VERTEX), "position")
        glBindAttribLocation(_program, GLuint(ATTRIB_TEXCOORD), "texCoord")

        // Link program.
        guard linkProgram(_program) else
        {
            print("Failed to link program: \(_program)")

            if vertShader != 0 {
                glDeleteShader(vertShader)
                vertShader = 0
            }
            if fragShader != 0 {
                glDeleteShader(fragShader)
                fragShader = 0
            }
            if _program != 0 {
                glDeleteProgram(_program)
                _program = 0
            }

            return false
        }

        // Get uniform locations.
        uniforms[UNIFORM_Y] = glGetUniformLocation(_program, "SamplerY")
        uniforms[UNIFORM_UV] = glGetUniformLocation(_program, "SamplerUV")

        // Release vertex and fragment shaders.
        if vertShader != 0 {
            glDetachShader(_program, vertShader)
            glDeleteShader(vertShader)
        }
        if fragShader != 0 {
            glDetachShader(_program, fragShader)
            glDeleteShader(fragShader)
        }

        return true
    }
}
