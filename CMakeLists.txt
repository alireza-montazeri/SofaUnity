cmake_minimum_required(VERSION 3.12)
project(SofaUnity VERSION 0.1)

# SOFA packages containing our dependencies
find_package(SofaGui REQUIRED)

# Are all these modules (formerly SofaGeneral) necessary?
find_package(SofaBoundaryCondition REQUIRED)
find_package(SofaConstraint REQUIRED)
find_package(SofaGeneralAnimationLoop REQUIRED)
find_package(SofaGeneralDeformable REQUIRED)
find_package(SofaGeneralEngine REQUIRED)
find_package(SofaGeneralExplicitOdeSolver REQUIRED)
find_package(SofaGeneralImplicitOdeSolver REQUIRED)
find_package(SofaGeneralLinearSolver REQUIRED)
find_package(SofaGeneralLoader REQUIRED)
find_package(SofaGeneralMeshCollision REQUIRED)
find_package(SofaGeneralObjectInteraction REQUIRED)
find_package(SofaGeneralRigid REQUIRED)
find_package(SofaGeneralSimpleFem REQUIRED)
find_package(SofaGeneralTopology REQUIRED)
find_package(SofaGeneralVisual REQUIRED)
find_package(SofaGraphComponent REQUIRED)
find_package(SofaTopologyMapping REQUIRED)
find_package(SofaUserInteraction REQUIRED)

find_package(SceneCreator REQUIRED)
find_package(SofaValidation REQUIRED)
find_package(SofaPhysicsAPI REQUIRED)

sofa_find_package(SofaComponentAll REQUIRED)
sofa_find_package(SofaSimulationCommon REQUIRED)

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
target_link_libraries(${PROJECT_NAME} PUBLIC SofaGui SceneCreator SofaValidation SofaSimulationCommon)
target_link_libraries(${PROJECT_NAME} PUBLIC
    SofaBoundaryCondition
    SofaConstraint
    SofaGeneralAnimationLoop
    SofaGeneralDeformable
    SofaGeneralEngine
    SofaGeneralExplicitOdeSolver
    SofaGeneralImplicitOdeSolver
    SofaGeneralLinearSolver
    SofaGeneralLoader
    SofaGeneralMeshCollision
    SofaGeneralObjectInteraction
    SofaGeneralRigid
    SofaGeneralSimpleFem
    SofaGeneralTopology
    SofaGeneralVisual
    SofaGraphComponent
    SofaTopologyMapping
    SofaUserInteraction
    SofaPhysicsAPI
    SofaComponentAll 
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

