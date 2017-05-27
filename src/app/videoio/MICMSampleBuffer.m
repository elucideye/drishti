/*
  MICMSampleBuffer.m
  MovieMaker

  Copyright (c) 2015 Zukini Ltd. All rights reserved.

   https://github.com/SheffieldKevin/VideoSample

 ------------------------------------------------------------------------------
 
 The MIT License (MIT)
 
 Copyright (c) 2015 Zukini Ltd.
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

#import "MICMSampleBuffer.h"

@implementation MICMSampleBuffer
{
    CMSampleBufferRef _ownedBuffer;
}

#pragma mark MICGImage initialization, copying and dealloc methods

-(instancetype)initWithMICMSampleBuffer:(MICMSampleBuffer *)sampleBuffer
{
    self = [super init];
    if (self)
    {
        CMSampleBufferRef inbuf = sampleBuffer.CMSampleBuffer;
        if (inbuf)
        {
            self->_ownedBuffer = (CMSampleBufferRef)CFRetain(inbuf);
        }
    }
    return self;
}

-(instancetype)initWithCMSampleBuffer:(CMSampleBufferRef)buffer
{
    self = [super init];
    if (self)
    {
        if (buffer)
        {
            self->_ownedBuffer = (CMSampleBufferRef)CFRetain(buffer);
        }
    }
    return self;
}

-(instancetype)copy
{
    return [[MICMSampleBuffer alloc] initWithMICMSampleBuffer:self];
}

-(void)dealloc
{
    if (self->_ownedBuffer)
    {
        CFRelease(self->_ownedBuffer);
    }
}

# pragma mark Manual property implementation

-(CMSampleBufferRef)CMSampleBuffer
{
    return self->_ownedBuffer;
}

-(void)setCMSampleBuffer:(CMSampleBufferRef)buffer
{
    if (buffer == self->_ownedBuffer)
        return;
    
    if (self->_ownedBuffer)
    {
        CFRelease(self->_ownedBuffer);
    }
    
    self->_ownedBuffer = NULL;
    if (buffer)
    {
        self->_ownedBuffer = (CMSampleBufferRef)CFRetain(buffer);
    }
}

#pragma mark Conforming to NSCopying protocol methods

-(instancetype)copyWithZone:(NSZone *)zone
{
    return [self copy];
}

@end
