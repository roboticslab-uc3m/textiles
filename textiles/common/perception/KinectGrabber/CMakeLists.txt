option(ENABLE_KinectGrabber "Compile freenect-based Kinect grabber" FALSE)

if (ENABLE_KinectGrabber)
     # Include libfreenect library from openkinect repository at https://github.com/OpenKinect/libfreenect/
    find_path(LIBFREENECT_INCLUDE libfreenect.h)
    find_library(LIBFREENECT_LIBRARIES freenect ${LIBFREENECT_INCLUDE}/../lib ${LIBFREENECT_INCLUDE}/../build/lib)
    include_directories(${LIBFREENECT_INCLUDE})

    # The viewer need pthreads and GLUT.
    set(THREADS_USE_PTHREADS_WIN32 true)
    find_package(Threads)
    find_package(OpenGL)
    find_package(GLUT)

    if (Threads_FOUND AND OPENGL_FOUND AND GLUT_FOUND)
    include_directories(${THREADS_PTHREADS_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
    # Define math lib
    if (WIN32)
      set(MATH_LIB "")
    else(WIN32)
      set(MATH_LIB "m")
    endif()

    add_executable(kinectGrabber kinectGrabber.c)
    target_link_libraries(kinectGrabber m ${LIBFREENECT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB})

    install(TARGETS kinectGrabber
            DESTINATION bin)
    
    endif()
endif()
