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

extern "C"
{
    SOFA_UNITY_EXPORT void createSofaUnityAPI();
    SOFA_UNITY_EXPORT const char *APIName();

    SOFA_UNITY_EXPORT bool load(const char *cfilename);
    void updateOutputMeshes();

    SOFA_UNITY_EXPORT double getTimeStep();
    SOFA_UNITY_EXPORT void setTimeStep(double dt);

    SOFA_UNITY_EXPORT double getTime();
    SOFA_UNITY_EXPORT double getCurrentFPS();

    SOFA_UNITY_EXPORT double *getGravity();
    SOFA_UNITY_EXPORT void setGravity(double *gravity);

    SOFA_UNITY_EXPORT void step();

    SOFA_UNITY_EXPORT unsigned int getNbMeshes();
    SOFA_UNITY_EXPORT unsigned int getNbMeshVertices(int mIndex);
    SOFA_UNITY_EXPORT int getMeshVPositions(float *vPositions, int mIndex);
}

#endif // SOFA_TEARPLUGIN_H
