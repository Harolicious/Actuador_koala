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
                rootNode.addObject("QPInverseProblemSolver", printLog=1, epsilon=0.1, maxIterations=1000,tolerance=1e-5)
                rootNode.dt = 0.001

		#body
                body = rootNode.addChild('body')
                body.addObject('EulerImplicitSolver', name='odesolver')
                
                body.addObject('SparseLDLSolver', name='preconditioner')

                body.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioner='@preconditioner', use_precond=True, update_step=1)

                Loader = body.addObject('MeshVTKLoader', name='loader', filename='Uniaxial.vtk')
                Container = body.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                body.addObject('TetrahedronSetTopologyModifier')

                MO = body.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False)
                body.addObject('UniformMass', totalMass=0.5)
                
                boxROIStiffness = body.addObject('BoxROI', name='boxROIStiffness', box=[-45, -45, 0,  45, 45, 5], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
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
                #body.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=180000)
                body.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=YMArray.flatten().tolist())
                #body.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM2', method='large', poissonRatio=0.3,  youngModulus=180000)
                
                #body.addObject('TetrahedronHyperelasticityFEMForceField', name="HyperElasticMaterial", materialName="MooneyRivlin", ParameterSet="48000 -1.5e5 3000")

                body.addObject('BoxROI', name='boxROI', box=[-45, -45, 150,  45, 45, 155], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                body.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
                body.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')
                #body.addObject('UncoupledConstraintCorrection')
                

               
                
		#cavity
                
        
                cavity = body.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='Uniaxial_cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')             
                SPA = cavity.addObject('SurfacePressureActuator', triangles='@topo.triangles')
                #cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0.5, valueType=0)
                # SPC = cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=0, valueType=0)
                #cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=False)
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=True, mapMasses=True)

        # Effector
        # bunny/effector
        # goal
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO', position=[60 , 0 , -5], showObject=True, showObjectScale=15)
                goal.addObject('SphereCollisionModel', radius=2.5, group=1)
                goal.addObject('UncoupledConstraintCorrection')
                
        # Punto "End-effector"         
                effector = body.addChild('EffectorNode')
                effector.addObject('MechanicalObject', position=[0, 0, 0], showObject=True, showObjectScale=10)
                PositionEffector = effector.addObject('PositionEffector', indices=0, effectorGoal=goal.goalMO.position.linkpath)
                effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

		#body/bodyVisu
                bodyVisu = body.addChild('visu')
                bodyVisu.addObject("MeshSTLLoader", filename="Uniaxial_visu.stl", name="loader")
                bodyVisu.addObject("OglModel", src="@loader")
                bodyVisu.addObject("BarycentricMapping")

                # rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SPC=SPC))
            

                return rootNode
