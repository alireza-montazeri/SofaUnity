cmake_minimum_required(VERSION 3.12)
project(SofaUnity VERSION 0.1)

# SOFA packages containing our dependencies
find_package(SofaPhysicsAPI REQUIRED)

# Config files
set(CONFIG_FILES
    src/config.h
    src/initUnityPlugin.cpp
    )
# Components
set(HEADER_FILES
    src/UnityPlugin.h
    )
set(SOURCE_FILES
    src/UnityPlugin.cpp
    )

# SOFA plugin are basically libraries
add_library(${PROJECT_NAME} SHARED ${CONFIG_FILES} ${HEADER_FILES} ${SOURCE_FILES})

# Dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
	SofaPhysicsAPI
    )

# nested-namespace-definition requires compiler flag '/std:c++17'
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})

# Create package Config, Version & Target files.
# Deploy the headers, resources, scenes & examples.
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    PACKAGE_VERSION ${PROJECT_VERSION}
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )

