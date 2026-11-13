
cmake_minimum_required(VERSION 3.20)

project(CerealPhysics
    VERSION 1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


include(FetchContent)

FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
)

FetchContent_MakeAvailable(glm)



# Main executable
add_executable(CerealPhysics
    src/main.cpp

    # Physics
    src/physics/RigidBody.cpp
    src/physics/PhysicsWorld.cpp
    src/physics/Integrator.cpp

    # Collision
    src/collision/AABB.cpp
    src/collision/CollisionDetection.cpp
    src/collision/CollisionResponse.cpp
)


# Directories
target_include_directories(CerealPhysics
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(CerealPhysics PRIVATE glm::glm)
# Compiler warnings
if(MSVC)
    target_compile_options(CerealPhysics PRIVATE /W4)
else()
    target_compile_options(CerealPhysics PRIVATE -Wall -Wextra -Wpedantic)
endif()
