/*!
  @file OpenGLRendering.swift
  @brief Common OpenGL swift functions for texture handling.

  \copyright Copyright 2018 Elucideye, Inc. All rights reserved. [All modifications]
  \license{This project is released under the 3 Clause BSD License.} [All modifications]

  Lineage:

  Implementations copied from https://github.com/BradLarson/GPUImage2 (BSD 3-Clause)

  https://github.com/BradLarson/GPUImage2/blob/master/License.txt
*/

import AVFoundation
import GLKit
import UIKit

func checkGLError()
{
    let error: GLenum = glGetError()
    if error != GLenum(GL_NO_ERROR)
    {
        fatalError("GL Error \(error)")
    }
}

struct GLSize
{
    let width: GLint
    let height: GLint
}

func crashOnShaderCompileFailure<T>(_ shaderName: String, _ operation: () throws -> T) -> T
{
    do
    {
        return try operation()
    }
    catch
    {
        fatalError("\(shaderName) compilation failed with error: \(error)")
    }
}

extension String
{
    func withGLChar(_ operation: (UnsafePointer<GLchar>) -> Void)
    {
        if let value = self.cString(using: String.Encoding.utf8)
        {
            operation(UnsafePointer<GLchar>(value))
        }
        else
        {
            fatalError("Could not convert this string to UTF8: \(self)")
        }
    }
}

func generateTexture(minFilter: Int32, magFilter: Int32, wrapS: Int32, wrapT: Int32) -> GLuint
{
    var texture: GLuint = 0

    glActiveTexture(GLenum(GL_TEXTURE1))
    glGenTextures(1, &texture)
    glBindTexture(GLenum(GL_TEXTURE_2D), texture)
    glTexParameteri(GLenum(GL_TEXTURE_2D), GLenum(GL_TEXTURE_MIN_FILTER), minFilter)
    glTexParameteri(GLenum(GL_TEXTURE_2D), GLenum(GL_TEXTURE_MAG_FILTER), magFilter)
    glTexParameteri(GLenum(GL_TEXTURE_2D), GLenum(GL_TEXTURE_WRAP_S), wrapS)
    glTexParameteri(GLenum(GL_TEXTURE_2D), GLenum(GL_TEXTURE_WRAP_T), wrapT)

    glBindTexture(GLenum(GL_TEXTURE_2D), 0)

    return texture
}

struct FramebufferCreationError: Error
{
    let errorCode: GLenum
}

func attachStencilBuffer(width: GLint, height: GLint)
    throws -> GLuint
{
    var stencilBuffer: GLuint = 0

    checkGLError()
    glGenRenderbuffers(1, &stencilBuffer)
    checkGLError()
    glBindRenderbuffer(GLenum(GL_RENDERBUFFER), stencilBuffer)

    // iOS seems to only support combination depth + stencil, from references
    checkGLError()
    glRenderbufferStorage(GLenum(GL_RENDERBUFFER), GLenum(GL_DEPTH24_STENCIL8), width, height)
    #if os(iOS)
        checkGLError()
        glFramebufferRenderbuffer(GLenum(GL_FRAMEBUFFER), GLenum(GL_DEPTH_ATTACHMENT), GLenum(GL_RENDERBUFFER), stencilBuffer)
    #endif
    checkGLError()
    glFramebufferRenderbuffer(GLenum(GL_FRAMEBUFFER), GLenum(GL_STENCIL_ATTACHMENT), GLenum(GL_RENDERBUFFER), stencilBuffer)

    checkGLError()
    glBindRenderbuffer(GLenum(GL_RENDERBUFFER), 0)

    checkGLError()
    let status = glCheckFramebufferStatus(GLenum(GL_FRAMEBUFFER))
    if status != GLenum(GL_FRAMEBUFFER_COMPLETE)
    {
        throw FramebufferCreationError(errorCode: status)
    }

    return stencilBuffer
}

func generateFramebufferForTexture(_ texture: GLuint, width: GLint, height: GLint, internalFormat: Int32, format: Int32, type: Int32, stencil: Bool)
    throws -> (GLuint, GLuint?)
{
    var framebuffer: GLuint = 0

    checkGLError()

    glActiveTexture(GLenum(GL_TEXTURE1))
    checkGLError()
    glGenFramebuffers(1, &framebuffer)
    checkGLError()
    glBindFramebuffer(GLenum(GL_FRAMEBUFFER), framebuffer)
    checkGLError()
    glBindTexture(GLenum(GL_TEXTURE_2D), texture)
    checkGLError()
    glTexImage2D(GLenum(GL_TEXTURE_2D), 0, internalFormat, width, height, 0, GLenum(format), GLenum(type), nil)
    checkGLError()
    glFramebufferTexture2D(GLenum(GL_FRAMEBUFFER), GLenum(GL_COLOR_ATTACHMENT0), GLenum(GL_TEXTURE_2D), texture, 0)
    checkGLError()

    let status = glCheckFramebufferStatus(GLenum(GL_FRAMEBUFFER))
    if status != GLenum(GL_FRAMEBUFFER_COMPLETE)
    {
        throw FramebufferCreationError(errorCode: status)
    }

    let stencilBuffer: GLuint?
    if stencil
    {
        stencilBuffer = try attachStencilBuffer(width: width, height: height)
        checkGLError()
    }
    else
    {
        stencilBuffer = nil
    }

    glBindTexture(GLenum(GL_TEXTURE_2D), 0)
    checkGLError()
    glBindFramebuffer(GLenum(GL_FRAMEBUFFER), 0)
    checkGLError()

    return (framebuffer, stencilBuffer)
}

// ::: SHADERS :::

func shaderFromFile(_ file: URL) throws -> String
{
    // Note: this is a hack until Foundation's String initializers are fully functional
    //        let fragmentShaderString = String(contentsOfURL:fragmentShaderFile, encoding:NSASCIIStringEncoding)
    guard FileManager.default.fileExists(atPath: file.path) else
    {
        throw ShaderCompileError(compileLog: "Shader file \(file) missing")
    }

    let fragmentShaderString = try NSString(contentsOfFile: file.path, encoding: String.Encoding.ascii.rawValue)

    return String(describing: fragmentShaderString)
}

func compileShader(_ shader: UnsafeMutablePointer<GLuint>, type: GLenum, url: URL) -> Bool
{
    var status: GLint = 0
    guard let shaderString = try? shaderFromFile(url) else { return false }

    shader.pointee = glCreateShader(type)
    shaderString.withGLChar
    { glString in
        var tempString: UnsafePointer<GLchar>? = glString
        glShaderSource(shader.pointee, 1, &tempString, nil)
        glCompileShader(shader.pointee)
    }

    // #if DEBUG
    var logLength: GLint = 0
    glGetShaderiv(shader.pointee, GLenum(GL_INFO_LOG_LENGTH), &logLength)
    if logLength > 0 {
        var log: [GLchar] = Array(repeating: 0, count: Int(logLength))
        glGetShaderInfoLog(shader.pointee, logLength, &logLength, &log)
        print("Shader compile log:\n\(String(cString: log))")
    }
    // #endif

    glGetShaderiv(shader.pointee, GLenum(GL_COMPILE_STATUS), &status)
    guard status != 0 else
    {
        glDeleteShader(shader.pointee)
        return false
    }

    return true
}

func linkProgram(_ prog: GLuint) -> Bool
{
    var status: GLint = 0
    glLinkProgram(prog)

    #if DEBUG
        var logLength: GLint = 0
        glGetProgramiv(prog, GLenum(GL_INFO_LOG_LENGTH), &logLength)
        if logLength > 0 {
            var log: [GLchar] = Array(repeating: 0, count: Int(logLength))
            glGetProgramInfoLog(prog, logLength, &logLength, &log)
            print("Program link log:\n\(String(cString: log))")
        }
    #endif

    glGetProgramiv(prog, GLenum(GL_LINK_STATUS), &status)
    if status == 0 {
        return false
    }

    return true
}
