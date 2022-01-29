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
 * Contact information: alireza.montazeri9675@gmail.com             	       *
 ******************************************************************************/

#define SOFA_UNITYPLUGIN_CPP

#include <UnityPlugin.h>

#include <sofa/gl/gl.h>
#include <sofa/gl/glu.h>
#include <sofa/helper/io/Image.h>
#include <sofa/gl/RAII.h>

#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/helper/system/PluginManager.h>
#include <sofa/helper/BackTrace.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/GUIEvent.h>

#include <SofaSimulationGraph/DAGSimulation.h>

#include <sofa/gui/GUIManager.h>
#include <SofaGui/initSofaGui.h>
#include <sofa/helper/init.h>

#include <sofa/gui/BaseGUI.h>

#include <sofa/type/Vec.h>

#include <cmath>
#include <iostream>

#include <SceneCreator/SceneCreator.h>

#include <SofaBoundaryCondition/initSofaBoundaryCondition.h>
#include <SofaConstraint/initSofaConstraint.h>
#include <SofaGeneralAnimationLoop/initSofaGeneralAnimationLoop.h>
#include <SofaGeneralDeformable/initSofaGeneralDeformable.h>
#include <SofaGeneralEngine/initSofaGeneralEngine.h>
#include <SofaGeneralExplicitOdeSolver/initSofaGeneralExplicitOdeSolver.h>
#include <SofaGeneralImplicitOdeSolver/initSofaGeneralImplicitOdeSolver.h>
#include <SofaGeneralLinearSolver/initSofaGeneralLinearSolver.h>
#include <SofaGeneralLoader/initSofaGeneralLoader.h>
#include <SofaGeneralMeshCollision/initSofaGeneralMeshCollision.h>
#include <SofaGeneralObjectInteraction/initSofaGeneralObjectInteraction.h>
#include <SofaGeneralRigid/initSofaGeneralRigid.h>
#include <SofaGeneralSimpleFem/initSofaGeneralSimpleFem.h>
#include <SofaGeneralTopology/initSofaGeneralTopology.h>
#include <SofaGeneralVisual/initSofaGeneralVisual.h>
#include <SofaGraphComponent/initSofaGraphComponent.h>
#include <SofaTopologyMapping/initSofaTopologyMapping.h>
#include <SofaUserInteraction/initSofaUserInteraction.h>
#include <SofaOpenglVisual/OglModel.h>

#include <SofaSimulationGraph/init.h>
#include <SofaSimulationCommon/init.h>

#include <sofa/helper/system/thread/CTime.h>

#include <fstream>

std::vector<sofa::component::visualmodel::OglModel *> outputMeshes;
sofa::simulation::Node::SPtr m_RootNode;
sofa::simulation::Simulation *m_Simulation;

sofa::helper::system::thread::ctime_t stepTime[10];
sofa::helper::system::thread::ctime_t timeTicks;
sofa::helper::system::thread::ctime_t lastRedrawTime;
int frameCounter;
double currentFPS;

SOFA_UNITY_EXPORT int createSofaUnityAPI(const char *sofaDir)
{
	/* Clear LOG file */
	remove("E:/MasterProject/sofa/log.txt");

	sofa::helper::init();
	sofa::simulation::common::init();
	sofa::simulation::graph::init();

	sofa::gui::GUIManager::Init(nullptr, "fake");
	sofa::gui::GUIManager::createGUI(NULL, NULL);

	m_Simulation = new sofa::simulation::graph::DAGSimulation();
	sofa::simulation::setSimulation(m_Simulation);

	sofa::component::initSofaBoundaryCondition();
	sofa::component::initSofaConstraint();
	sofa::component::initSofaGeneralAnimationLoop();
	sofa::component::initSofaGeneralDeformable();
	sofa::component::initSofaGeneralEngine();
	sofa::component::initSofaGeneralExplicitOdeSolver();
	sofa::component::initSofaGeneralImplicitOdeSolver();
	sofa::component::initSofaGeneralLinearSolver();
	sofa::component::initSofaGeneralLoader();
	sofa::component::initSofaGeneralMeshCollision();
	sofa::component::initSofaGeneralObjectInteraction();
	sofa::component::initSofaGeneralRigid();
	sofa::component::initSofaGeneralSimpleFem();
	sofa::component::initSofaGeneralTopology();
	sofa::component::initSofaGeneralVisual();
	sofa::component::initSofaGraphComponent();
	sofa::component::initSofaTopologyMapping();
	sofa::component::initSofaUserInteraction();

	sofa::helper::system::PluginManager::getInstance().init();

	std::string dir = sofaDir;
	sofa::helper::system::PluginRepository.clear();
	sofa::helper::system::PluginRepository.addFirstPath(dir + "/bin");
	sofa::helper::system::PluginRepository.addFirstPath(dir + "/plugins");
	sofa::helper::system::PluginRepository.addFirstPath(dir + "/lib");

	timeTicks = sofa::helper::system::thread::CTime::getRefTicksPerSec();
	frameCounter = 0;
	currentFPS = 0.0;
	lastRedrawTime = 0;

	return 0;
}

SOFA_UNITY_EXPORT const char *APIName()
{
	return "SofaUnity API";
}

SOFA_UNITY_EXPORT int load(const char *cfilename)
{
	int success = 0;

	std::string filename = cfilename;
	std::cout << "FROM APP: SofaUnity load(" << filename << ")" << std::endl;
	sofa::helper::BackTrace::autodump();

	int idx = (int)filename.find_last_of("/");
	std::string name = filename.substr(idx + 1);
	std::string dir = filename.substr(0, idx);
	sofa::helper::system::DataRepository.addFirstPath(dir);
	sofa::helper::system::DataRepository.findFile(name);

	std::ofstream logFile;
	logFile.open("E:/MasterProject/sofa/log.txt", std::ios::app);
	logFile << name << std::endl;
	logFile.close();

	m_RootNode = m_Simulation->load(name.c_str());
	if (m_RootNode.get())
	{
		m_Simulation->init(m_RootNode.get());
		updateOutputMeshes();
	}
	else
	{
		std::cerr << "Error: can't get m_RootNode" << std::endl;
		success = -1;
	}
	lastRedrawTime = sofa::helper::system::thread::CTime::getRefTime();
	return success;
}

void updateOutputMeshes()
{
	outputMeshes.clear();

	// Get list of all 3D models
	m_RootNode->getTreeObjects<sofa::component::visualmodel::OglModel>(&outputMeshes);
}

SOFA_UNITY_EXPORT double getTimeStep()
{
	if (m_RootNode.get())
		return m_RootNode.get()->getContext()->getDt();
	else
		return 0.0;
}

SOFA_UNITY_EXPORT void setTimeStep(double dt)
{
	if (m_RootNode.get())
	{
		m_RootNode.get()->getContext()->setDt(dt);
	}
}

SOFA_UNITY_EXPORT double getTime()
{
	if (m_RootNode.get())
		return m_RootNode.get()->getContext()->getTime();
	else
		return 0.0;
}

SOFA_UNITY_EXPORT double getCurrentFPS()
{
	return currentFPS;
}

SOFA_UNITY_EXPORT double *getGravity()
{
	double *gravityVec = new double[3];

	if (m_RootNode.get())
	{
		const auto &g = m_RootNode.get()->getContext()->getGravity();
		gravityVec[0] = g.x();
		gravityVec[1] = g.y();
		gravityVec[2] = g.z();
	}

	return gravityVec;
}

SOFA_UNITY_EXPORT void setGravity(double *gravity)
{
	const auto &g = sofa::type::Vec3d(gravity[0], gravity[1], gravity[2]);
	m_RootNode.get()->getContext()->setGravity(g);
}

void updateCurrentFPS()
{
	if (frameCounter == 0)
	{
		sofa::helper::system::thread::ctime_t t = sofa::helper::system::thread::CTime::getRefTime();
		for (int i = 0; i < 10; i++)
			stepTime[i] = t;
	}
	else
	{
		if ((frameCounter % 10) == 0)
		{
			sofa::helper::system::thread::ctime_t curtime = sofa::helper::system::thread::CTime::getRefTime();
			int i = ((frameCounter / 10) % 10);
			currentFPS = ((double)timeTicks / (curtime - stepTime[i])) * (frameCounter < 100 ? frameCounter : 100);
			stepTime[i] = curtime;
		}
	}
	++frameCounter;
}

SOFA_UNITY_EXPORT void step()
{
	sofa::simulation::Node *groot = m_RootNode.get();
	if (!groot)
		return;
	/* Begin Step */
	m_Simulation->animate(groot);
	m_Simulation->updateVisual(groot);

	/* End Step */
	updateCurrentFPS();
	updateOutputMeshes();
}

SOFA_UNITY_EXPORT unsigned int getNbMeshes()
{
	return (unsigned int)outputMeshes.size();
}

SOFA_UNITY_EXPORT unsigned int getNbMeshVertices(int mIndex)
{
	if (mIndex >= outputMeshes.size())
		return 0;

	return (unsigned int)outputMeshes[mIndex]->getVertices().size();
}

SOFA_UNITY_EXPORT int getMeshVPositions(float *&vPositions, int mIndex)
{
	// Get array of vertices from sofa
	sofa::component::visualmodel::VisualModelImpl::VecCoord meshVPositions = outputMeshes[mIndex]->getVertices();

	unsigned int nbVertices = (unsigned int)meshVPositions.size();
	unsigned int length = 3 * nbVertices;

	for (unsigned int i = 0; i < nbVertices; i++)
	{
		vPositions[3 * i] = (float)meshVPositions[i].x();
		vPositions[3 * i + 1] = (float)meshVPositions[i].y();
		vPositions[3 * i + 2] = (float)meshVPositions[i].z();
	}

	return 0;
}

SOFA_UNITY_EXPORT unsigned int getNbMeshTriangles(int mIndex)
{
	unsigned int nbTriangles = 0;
	if (mIndex >= outputMeshes.size())
		return 0;
	nbTriangles = (unsigned int)outputMeshes[mIndex]->getTriangles().size() + 2 * (unsigned int)outputMeshes[mIndex]->getQuads().size();

	return nbTriangles;
}

SOFA_UNITY_EXPORT int getMeshTriangles(int *&triangles, int mIndex)
{
	// Get array of tangents from sofa
	const sofa::component::visualmodel::VisualModelImpl::VecVisualTriangle meshTriangles = outputMeshes[mIndex]->getTriangles();
	unsigned int nbTriangles = (unsigned int)meshTriangles.size();

	// Fill triangles_ array
	for (unsigned int i = 0; i < nbTriangles; i++)
	{
		triangles[3 * i] = meshTriangles[i][0];
		triangles[3 * i + 1] = meshTriangles[i][1];
		triangles[3 * i + 2] = meshTriangles[i][2];
	}

	const sofa::component::visualmodel::VisualModelImpl::VecVisualQuad meshQuads = outputMeshes[mIndex]->getQuads();
	unsigned int nbQuads = (unsigned int)meshQuads.size();

	for (unsigned int i = 0; i < nbQuads; i++)
	{
		unsigned int index = nbTriangles + 6 * i;
		triangles[index] = meshQuads[i][0];
		triangles[index + 1] = meshQuads[i][1];
		triangles[index + 2] = meshQuads[i][2];

		triangles[index + 3] = meshQuads[i][2];
		triangles[index + 4] = meshQuads[i][3];
		triangles[index + 5] = meshQuads[i][0];
	}

	return 0;
}

SOFA_UNITY_EXPORT int getMeshTranslation(float *&translation, int mIndex)
{
	sofa::component::visualmodel::VisualModelImpl::Vec3Real translationVec = outputMeshes[mIndex]->m_translation.getValue();
	translation[0] = (float)translationVec[0];
	translation[1] = (float)translationVec[1];
	translation[2] = (float)translationVec[2];

	return 0;
}

SOFA_UNITY_EXPORT int getMeshRotation(float *&rotation, int mIndex)
{
	sofa::component::visualmodel::VisualModelImpl::Vec3Real rotationVec = outputMeshes[mIndex]->m_rotation.getValue();
	rotation[0] = (float)rotationVec[0];
	rotation[1] = (float)rotationVec[1];
	rotation[2] = (float)rotationVec[2];

	return 0;
}

SOFA_UNITY_EXPORT int getMeshScale(float *&scale, int mIndex)
{
	sofa::component::visualmodel::VisualModelImpl::Vec3Real scaleVec = outputMeshes[mIndex]->m_scale.getValue();
	scale[0] = (float)scaleVec[0];
	scale[1] = (float)scaleVec[1];
	scale[2] = (float)scaleVec[2];

	return 0;
}

SOFA_UNITY_EXPORT int getMeshColor(float *&color, int mIndex)
{
	if (outputMeshes[mIndex]->material.getValue().useDiffuse)
	{
		auto rgba = outputMeshes[mIndex]->material.getValue().diffuse;

		color[0] = rgba[0];
		color[1] = rgba[1];
		color[2] = rgba[2];
		color[3] = rgba[3];
	}
	else
	{
		color[0] = 1;
		color[1] = 1;
		color[2] = 1;
		color[3] = 1;
	}
	return 0;
}
