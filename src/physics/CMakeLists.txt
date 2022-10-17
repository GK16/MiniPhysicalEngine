add_executable(physics collisions.cpp main.cpp physics.cpp planebody.cpp
		spherebody.cpp spring.cpp trianglebody.cpp)
target_link_libraries(physics application math scene tinyxml ${SDL_LIBRARY}
                      ${PNG_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES}
                      ${GLEW_LIBRARIES})

if(APPLE)
    target_link_libraries(physics SDLmain)
endif()

install(TARGETS physics DESTINATION ${PROJECT_SOURCE_DIR}/..)
