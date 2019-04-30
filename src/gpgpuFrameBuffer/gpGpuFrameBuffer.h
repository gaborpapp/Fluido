//
//  gpGpuFrameBuffer.h
//
//  The MIT License (MIT)
//  Copyright (c) 2015 Luca Lolli.
//  Created by luca lolli on 16/12/2015.
//
//

#ifndef gpGpuFrameBuffer_h
#define gpGpuFrameBuffer_h


#include "cinder/cinder.h"
#include "cinder/app/App.h"
#include "cinder/gl/gl.h"

namespace ds {
    typedef std::shared_ptr<class gpGpuFrameBuffer> gpGpuFrameBufferRef;

    class gpGpuFrameBuffer{

    public:
        static gpGpuFrameBufferRef create(int width, int height, GLint colorFormat = GL_RGBA32F)
        {
            return gpGpuFrameBufferRef( new gpGpuFrameBuffer{width, height, colorFormat});
        }

        static gpGpuFrameBufferRef create(const ci::ivec2 &size, GLint colorFormat = GL_RGBA32F)
        {
            return gpGpuFrameBufferRef( new gpGpuFrameBuffer{size.x, size.y, colorFormat});
        }

        gpGpuFrameBuffer(int width, int height, GLint colorFormat = GL_RGBA32F);

        gpGpuFrameBuffer(const ci::ivec2 &size, GLint colorFormat = GL_RGBA32F );
        ~gpGpuFrameBuffer();

        void initFbo();

        void clear();

		ci::gl::FboRef getBuffer();

        GLint getBufferLocation();

        GLint getTextureLocation();

        void drawBuffer();

        void bindTexture(int textureUnit = 0);

        void unbindTexture();

        void bindBuffer();

        void unbindBuffer(bool toSwap = false);

		ci::gl::TextureRef getTexture();

        void swap();

        void draw();

		ci::ivec2 getSize();

    private:
        GLint                   mColorFormat;
        int                     mWidth, mHeight;

		ci::gl::Fbo::Format         mFboFormat;
		ci::gl::Texture::Format     mTextureFormat;

        GLenum                  mReadIndex, mWriteIndex;

		ci::gl::FboRef              mFbo;
    };
}
#endif /* gpGpuFrameBuffer_h */
