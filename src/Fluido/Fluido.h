//
//  Fluido.h
//  Fluido
//  The MIT License (MIT)
//  Copyright (c) 2015 Luca Lolli.
//  Created by luca lolli on 21/12/2015.
//
//

#ifndef Fluido_h
#define Fluido_h

#include "cinder/Cinder.h"
#include "cinder/gl/gl.h"
#include "gpGpuFrameBuffer.h"
#include "cinder/params/Params.h"
#include "cinder/ConcurrentCircularBuffer.h"

namespace ds {
    typedef std::shared_ptr<class Fluido> FluidoRef;

    struct impulsePoint {
		ci::vec2    position;
		ci::vec2    direction;
        float   magnitude;

		ci::ColorA  color;
        float   radius;
        float   force;

        bool addDens;
        bool addTemp;
        bool addVel;
    };

    class Fluido{
    public:
        static FluidoRef create(int width, int height){
            return FluidoRef( new Fluido{ ci::ivec2(width,height) });
        }
        static FluidoRef create(const ci::ivec2 &size){
            return FluidoRef( new Fluido{ size });
        }

        Fluido(int width, int height);

        Fluido(const ci::ivec2 &size);

        void resetObstacles(bool isContained = false);

        void loadShaders();
        //! Register the parameters to the cinder InterfaceGL
        void registerParams(const ci::params::InterfaceGlRef &params);
        //! Update the simulation, the Default value is the internal dT.
        void update(float deltaT = -1.0f);
        //! Draw a temporary circle into the Density and Temperature buffer, if magnetude > 0 draw also into the velocity texture with the corresponding direction value.
        void addImpulsePoint(impulsePoint point);
        //! Draw a circle every frame into the Density and Temperature buffer, if magnetude > 0 draw also into the velocity texture with the corresponding direction value.
        void addConstantImpulsePoint(impulsePoint point);
        //! Add obstacle texure (Need just RED channel)
        void addObstacle(const ci::gl::TextureRef &obstacle);
        //! Add Impulse Color, will be added to Density and Temperature buffer without any velocity
        void addImpulseTexture(const ci::gl::TextureRef &colorTexture);
        //! Add Impulse Color, will be added to Density and Temperature buffer and add Velocity Texture to be applied to the Velocity BUffer
        void addImpulseTexture(const ci::gl::TextureRef &colorTexture, const ci::gl::TextureRef &velocityTexture);

		ci::ivec2 getSize();

        void drawVelocity(const ci::vec2 &size);

        void drawDensity(const ci::vec2 &size);

        void drawObstacles(const ci::vec2 &size);

        void drawTemperature(const ci::vec2 &size);

        void drawPressure(const ci::vec2 &size);

        void drawVelocity(const ci::Rectf &bounds);

        void drawDensity(const ci::Rectf &bounds);

        void drawObstacles(const ci::Rectf &bounds);

        void drawTemperature(const ci::Rectf &bounds);

        void drawPressure(const ci::Rectf &bounds);

       ~Fluido();

    private:

        void initBuffers();

        void clear(const ci::gl::FboRef &fbo, const ci::ColorA &color = ci::ColorA(0.0,0.0,0.0,1.0));

        void Advect(const gpGpuFrameBufferRef &buffer, const ci::gl::TextureRef &velocity, const ci::gl::TextureRef &obstacles, float dissipation, float timeStep);

        void Buoyancy(const gpGpuFrameBufferRef &buffer, const ci::gl::TextureRef &temperature, const ci::gl::TextureRef &density, float AmbientTemperature, float TimeStep, float Kappa, float Sigma, const ci::vec3 &gravity = ci::vec3(0.0));

        void ComputeDivergence(const ci::gl::FboRef &fbo, const ci::gl::TextureRef &velocity, const ci::gl::TextureRef &obstacles, float cellSize);

        void Jacobi(const gpGpuFrameBufferRef &buffer, const ci::gl::TextureRef &divergence, const ci::gl::TextureRef &obstacles, float cellsize, float inverseBeta);

        void subtractGradient(const gpGpuFrameBufferRef &buffer, const ci::gl::TextureRef &pressure, const ci::gl::TextureRef &obstacles, float gradientScale);

        void vorticityFirst(const ci::gl::FboRef &buffer, const ci::gl::TextureRef& velocityTexture, const ci::gl::TextureRef& obstaclesTexture);

        void vorticitySecond(const ci::gl::FboRef &buffer, const ci::gl::TextureRef& vorticityTexture, float deltaT, float scale, float cellSize);

        void addForce(const gpGpuFrameBufferRef &buffer, const ci::gl::TextureRef &texture, float force);
        void clampTexture(const gpGpuFrameBufferRef &buffer, float maxValue, float clampingForce);

        void injectImpulse(const ci::gl::FboRef &fbo, const ci::vec2 &position, float radius, const ci::vec4 &color);
        void injectTexture(const gpGpuFrameBufferRef &buffer, const ci::gl::TextureRef &texture);

		ci::ivec2           mSize;

		ci::gl::GlslProgRef jacobiShader,
                            buoyancyShader,
                            divergenceShader,
                            advectShader,
                            subGradientShader,
                            impulseShader,
                            visualizeShader,
                            velVisualizerShader,
                            vorticityFirstShader,
                            vorticitySecondShader,
                            addForceShader,
                            clampTextureShader;


        gpGpuFrameBufferRef mVelocityBuffer,
                            mDensityBuffr,
                            mPressureBuffer,
                            mTemperatureBuffer;

		ci::gl::FboRef      mDivergenceFbo,
                            mObstaclesFbo,
                            mHiResObstaclesFbo,
                            mVorticityFirstPassFbo,
                            mVorticitySecondPassFbo;

		ci::gl::FboRef      mFbo;

        float               mDissipation,
                            mTimeStep,
                            mSigma,
                            mKappa,
                            mAmbientTemp,
                            mInversBeta,
                            mCellSize,
                            mGradientScale,
                            mVorticityForce,
                            mClampingForce,
                            mMaxVelocity,
                            mMaxDensity,
                            mMaxTemperature;

        int                 mNumIterations;

        float               mDeltaT;

        float               mPrevTime;

		ci::vec3            mGravity;

		ci::ConcurrentCircularBuffer<impulsePoint>	*mImpulsePoints;
		std::vector<impulsePoint>	mConstantImpulsePoints;

		ci::params::InterfaceGlRef  mParamRef;

		ci::gl::GlslProgRef mImpulseTextureShader;
		ci::gl::TextureRef  mImpulseColorTex;
		ci::gl::TextureRef  mImpulseVelocityTex;
        bool                mImpulseColorNew;
        bool                mImpulseVelocityNew;
    };
}
#endif /* Fluido_h */
