/******************************************************************************
 *                 SOFA, Simulation Open-Framework Architecture                *
 *                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
 *                                                                             *
 * This program is free software; you can redistribute it and/or modify it     *
 * under the terms of the GNU Lesser General Public License as published by    *
 * the Free Software Foundation; either version 2.1 of the License, or (at     *
 * your option) any later version.                                             *
 *                                                                             *
 * This program is distributed in the hope that it will be useful, but WITHOUT *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
 * for more details.                                                           *
 *                                                                             *
 * You should have received a copy of the GNU Lesser General Public License    *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.        *
 *******************************************************************************
 * Authors: Alireza Montazeri                                                  *
 *                                                                             *
 * Contact information: alireza.montazeri9675@gmail.com                        *
 ******************************************************************************/
#ifndef SOFA_UNITYPLUGIN_H
#define SOFA_UNITYPLUGIN_H

#define SOFA_UNITY_EXPORT __declspec(dllexport)

#include <SofaPhysicsAPI/SofaPhysicsAPI.h>

extern "C"
{
    SOFA_UNITY_EXPORT SofaPhysicsAPI *createAPI();
    SOFA_UNITY_EXPORT const char *APIName(SofaPhysicsAPI *instance);

    SOFA_UNITY_EXPORT bool load(SofaPhysicsAPI *instance, const char *filename);
    SOFA_UNITY_EXPORT void createScene(SofaPhysicsAPI *instance);

    SOFA_UNITY_EXPORT void start(SofaPhysicsAPI *instance);
    SOFA_UNITY_EXPORT void stop(SofaPhysicsAPI *instance);
    SOFA_UNITY_EXPORT void step(SofaPhysicsAPI *instance);
    SOFA_UNITY_EXPORT void reset(SofaPhysicsAPI *instance);
    SOFA_UNITY_EXPORT void resetView(SofaPhysicsAPI *instance);
    SOFA_UNITY_EXPORT void sendValue(SofaPhysicsAPI *instance, const char *name, double value);
    SOFA_UNITY_EXPORT void drawGL(SofaPhysicsAPI *instance);

    SOFA_UNITY_EXPORT double getCurrentFPS(SofaPhysicsAPI *instance);

    SOFA_UNITY_EXPORT unsigned int getNbOutputMeshes(SofaPhysicsAPI *instance);

    SOFA_UNITY_EXPORT bool isAnimated(SofaPhysicsAPI *instance);
    SOFA_UNITY_EXPORT void setAnimated(SofaPhysicsAPI *instance, bool val);

    SOFA_UNITY_EXPORT double getTimeStep(SofaPhysicsAPI *instance);
    SOFA_UNITY_EXPORT void setTimeStep(SofaPhysicsAPI *instance, double dt);
    SOFA_UNITY_EXPORT double getTime(SofaPhysicsAPI *instance);
    SOFA_UNITY_EXPORT double getCurrentFPS(SofaPhysicsAPI *instance);
    SOFA_UNITY_EXPORT double *getGravity(SofaPhysicsAPI *instance);
    SOFA_UNITY_EXPORT void setGravity(SofaPhysicsAPI *instance, double *gravity);
}

#endif // SOFA_TEARPLUGIN_H
