//  MICMSampleBuffer.m
//  MovieMaker
//
//  Copyright (c) 2015 Zukini Ltd. All rights reserved.

// https://github.com/SheffieldKevin/VideoSample

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
