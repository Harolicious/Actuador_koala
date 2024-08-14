#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 12 18:12:32 2024

@author: lab
"""

import Sofa

import os
import numpy as np
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

# class Controller(Sofa.Core.Controller):   
    
#     def __init__(self, *args, **kwargs):
#         Sofa.Core.Controller.__init__(self, *args, **kwargs)
#         print(" Python::__init__::" + str(self.name.value))
        
#         self.RootNode = kwargs['RootNode']
#         self.SPC = kwargs['SPC']
#         self.Increment = 0.4
#         self.Pressure = 0
#         print(kwargs['RootNode'])
    
        
#         print('Finished Init')
        
#     def onAnimateBeginEvent(self, eventType):
#         self.Pressure = self.Pressure + self.Increment
#         if self.Pressure > 60 or self.Pressure < 0:
#             # self.Pressure = 550
#             self.Increment = -self.Increment
#         self.SPC.value.value = [self.Pressure]
        
#         pass
  

def createScene(rootNode):

                rootNode.addObject(
                    "RequiredPlugin",
                    pluginName="""SofaPython3
                    SoftRobots
                    SoftRobots.Inverse
                    Sofa.Component.AnimationLoop
                    Sofa.Component.Constraint.Lagrangian.Correction
                    Sofa.Component.Constraint.Lagrangian.Solver
                    Sofa.Component.Engine.Select
                    Sofa.Component.IO.Mesh
                    Sofa.Component.LinearSolver.Direct
                    Sofa.Component.LinearSolver.Iterative
                    Sofa.Component.Mapping.Linear
                    Sofa.Component.Mapping.MappedMatrix
                    Sofa.Component.Mapping.NonLinear
                    Sofa.Component.Mass
                    Sofa.Component.ODESolver.Backward
                    Sofa.Component.Setting
                    Sofa.Component.SolidMechanics.FEM.Elastic
                    Sofa.Component.SolidMechanics.Spring
                    Sofa.Component.StateContainer
                    Sofa.Component.Topology.Container.Constant
                    Sofa.Component.Topology.Container.Dynamic
                    Sofa.Component.Visual
                    Sofa.GL.Component.Rendering3D
                    Sofa.GL.Component.Shader"""
                )
                
                rootNode.addObject(
                    "VisualStyle",
                    displayFlags="""
                        hideWireframe
                        showBehaviorModels
                        hideCollisionModels
                        hideBoundingCollisionModels
                        showForceFields
                        showInteractionForceFields""",
                )
                # rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
                rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Mapping') # Needed to use components [Tetra2TriangleTopologicalMapping]
                rootNode.addObject('FreeMotionAnimationLoop')
                rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance = 0.0000001)
                rootNode.dt = 0.0001

		#finger
                finger = rootNode.addChild('finger')
                finger.addObject('EulerImplicitSolver', name='odesolver')
                
                finger.addObject('SparseLDLSolver', name='preconditioner')

                finger.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioner='@preconditioner', use_precond=True, update_step=1)

                Loader = finger.addObject('MeshVTKLoader', name='loader', filename='Uniaxial.vtk')
                Container = finger.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                finger.addObject('TetrahedronSetTopologyModifier')

                MO = finger.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False)
                finger.addObject('UniformMass', totalMass=0.5)
                
                boxROIStiffness = finger.addObject('BoxROI', name='boxROIStiffness', box=[-45, -45, 0,  45, 45, 5], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                Container.init()
                MO.init()
                boxROIStiffness.init()
                YM1 = 180000
                YM2 = YM1*100
                YMArray = np.ones(len(Loader.tetras))*YM1
                IdxElementsInROI = np.array(boxROIStiffness.tetrahedronIndices.value)
                YMArray[IdxElementsInROI] = YM2
                print(f"len IdxElementsInROI: {len(IdxElementsInROI)}")
                
                print(f"Largo de YMArray:{len(YMArray)}")
                #finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=180000)
                finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=YMArray.flatten().tolist())
                #finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM2', method='large', poissonRatio=0.3,  youngModulus=180000)
                
                #finger.addObject('TetrahedronHyperelasticityFEMForceField', name="HyperElasticMaterial", materialName="MooneyRivlin", ParameterSet="48000 -1.5e5 3000")

                finger.addObject('BoxROI', name='boxROI', box=[-45, -45, 150,  45, 45, 155], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                finger.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                finger.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')
                #finger.addObject('UncoupledConstraintCorrection')
                

               
                
		#cavity
                
        
                cavity = finger.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='Uniaxial_cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')             
                cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0.5, valueType=0)
                # SPC = cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)
                #cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=False)
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)


		#finger/fingerVisu
                fingerVisu = finger.addChild('visu')
                fingerVisu.addObject("MeshSTLLoader", filename="Uniaxial_visu.stl", name="loader")
                fingerVisu.addObject("OglModel", src="@loader")
                fingerVisu.addObject("BarycentricMapping")

                # rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPC=SPC))
            

                return rootNode
