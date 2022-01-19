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
 * Contact information: alireza.montazeri9675@gmail.com             	           *
 ******************************************************************************/

#define SOFA_UNITYPLUGIN_CPP

#include <UnityPlugin.h>

extern "C"
{
	SOFA_UNITY_EXPORT SofaPhysicsAPI *createAPI()
	{
		return new SofaPhysicsAPI(false, 0);
	}

	SOFA_UNITY_EXPORT const char *APIName(SofaPhysicsAPI *instance)
	{
		return instance->APIName();
	}

	/// Load an XML file containing the main scene description
	SOFA_UNITY_EXPORT bool load(SofaPhysicsAPI *instance, const char *filename)
	{
		return instance->load(filename);
	}

	SOFA_UNITY_EXPORT void createScene(SofaPhysicsAPI *instance)
	{
		instance->createScene();
	}

	/// Start the simulation
	/// Currently this simply sets the animated flag to true, but this might
	/// start a separate computation thread in a future version
	SOFA_UNITY_EXPORT void start(SofaPhysicsAPI *instance)
	{
		instance->start();
	}

	/// Stop/pause the simulation
	SOFA_UNITY_EXPORT void stop(SofaPhysicsAPI *instance)
	{
		instance->stop();
	}

	/// Compute one simulation time-step
	SOFA_UNITY_EXPORT void step(SofaPhysicsAPI *instance)
	{
		instance->step();
	}

	/// Reset the simulation to its initial state
	SOFA_UNITY_EXPORT void reset(SofaPhysicsAPI *instance)
	{
		instance->reset();
	}

	/// Send an event to the simulation for custom controls
	/// (such as switching active instrument)
	SOFA_UNITY_EXPORT void sendValue(SofaPhysicsAPI *instance, const char *name, double value)
	{
		instance->sendValue(name, value);
	}

	/// Reset the camera to its default position
	SOFA_UNITY_EXPORT void resetView(SofaPhysicsAPI *instance)
	{
		instance->resetView();
	}

	/// Render the scene using OpenGL
	SOFA_UNITY_EXPORT void drawGL(SofaPhysicsAPI *instance)
	{
		instance->drawGL();
	}

	/// Return true if the simulation is running
	/// Note that currently you must call the step() method
	/// periodically to actually animate the scene
	SOFA_UNITY_EXPORT bool isAnimated(SofaPhysicsAPI *instance)
	{
		return instance->isAnimated();
	}

	/// Set the animated state to a given value (requires a
	/// simulation to be loaded)
	SOFA_UNITY_EXPORT void setAnimated(SofaPhysicsAPI *instance, bool val)
	{
		instance->setAnimated(val);
	}

	/// Return the main simulation file name (from the last
	/// call to load())
	SOFA_UNITY_EXPORT const char *getSceneFileName(SofaPhysicsAPI *instance)
	{
		return instance->getSceneFileName();
	}

	/// Return the current time-step (or 0 if no simulation
	/// is loaded)
	SOFA_UNITY_EXPORT double getTimeStep(SofaPhysicsAPI *instance)
	{
		return instance->getTimeStep();
	}

	/// Control the timestep of the simulation (requires a
	/// simulation to be loaded)
	SOFA_UNITY_EXPORT void setTimeStep(SofaPhysicsAPI *instance, double dt)
	{
		instance->setTimeStep(dt);
	}

	/// Return the current computation speed (averaged over
	/// the last 100 steps)
	SOFA_UNITY_EXPORT double getCurrentFPS(SofaPhysicsAPI *instance)
	{
		return instance->getCurrentFPS();
	}

	/// Return the number of currently active output meshes
	SOFA_UNITY_EXPORT unsigned int getNbOutputMeshes(SofaPhysicsAPI *instance)
	{
		return instance->getNbOutputMeshes();
	}

	SOFA_UNITY_EXPORT unsigned int getNbVertices(SofaPhysicsAPI *instance)
	{
		unsigned int nbMehses = instance->getNbOutputMeshes();
		return 0;
	}
}