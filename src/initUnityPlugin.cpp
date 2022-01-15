/******************************************************************************
 *       SOFA, Simulation Open-Framework Architecture, development version     *
 *                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
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
#include <config.h>

namespace sofa
{
    namespace component
    {

        extern "C"
        {
            SOFAUNITY_API
            void initExternalModule()
            {
                static bool first = true;
                if (first)
                {
                    first = false;
                }
            }

            SOFAUNITY_API
            const char *getModuleName()
            {
                return "SofaUnity";
            }

            SOFAUNITY_API
            const char *getModuleVersion()
            {
                return "0.1";
            }

            SOFAUNITY_API
            const char *getModuleLicense()
            {
                return "LGPL";
            }

            SOFAUNITY_API
            const char *getModuleDescription()
            {
                return "SOFA Unity plugin";
            }

            SOFAUNITY_API
            const char *getModuleComponentList()
            {
                // string containing the names of the classes provided by the plugin
                return "Unity";
            }

        } // extern "C"

    } // namespace component
} // namespace sofa
