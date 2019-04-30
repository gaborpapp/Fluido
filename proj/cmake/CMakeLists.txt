cmake_minimum_required( VERSION 3.0 FATAL_ERROR )
set( CMAKE_VERBOSE_MAKEFILE ON )

project( Fluido )
set( APP_NAME "${PROJECT_NAME}App" )

set( CMAKE_BUILD_TYPE "Release" )

get_filename_component( CINDER_PATH "../../../../" ABSOLUTE )

get_filename_component( APP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../" ABSOLUTE )

set( APP_INCS
	${APP_DIR}/include
	${APP_DIR}/src/Fluido
	${APP_DIR}/src/gpgpuFrameBuffer
)

set( APP_SRCS
	${APP_DIR}/src/FluidoApp.cpp
	${APP_DIR}/src/Fluido/Fluido.cpp
	${APP_DIR}/src/gpgpuFrameBuffer/gpGpuFrameBuffer.cpp
)

include( "${CINDER_PATH}/proj/cmake/modules/cinderMakeApp.cmake" )

ci_make_app(
	APP_NAME ${APP_NAME}
	INCLUDES ${APP_INCS}
	SOURCES ${APP_SRCS}
	CINDER_PATH ${CINDER_PATH}
)

if ( APPLE )
	get_target_property( OUTPUT_DIR ${APP_NAME} RUNTIME_OUTPUT_DIRECTORY )

	add_custom_target( run
		COMMAND open ${OUTPUT_DIR}/${APP_NAME}.app
		DEPENDS ${OUTPUT_DIR}/${APP_NAME}.app/Contents/MacOS/${APP_NAME}
		WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
	)
endif()
