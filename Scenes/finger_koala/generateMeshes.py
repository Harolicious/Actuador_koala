#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 12 17:52:31 2024

@author: lab
"""

import gmsh
from defineMeshSizes import defineMeshSizes

gmsh.initialize()

gmsh.merge("Unión Gripper 1 v2.step")
gmsh.model.occ.synchronize()
defineMeshSizes(4)
gmsh.model.mesh.generate(3)
# gmsh.model.mesh.refine()
gmsh.write("Finger.vtk")
gmsh.fltk.run()

gmsh.clear()

gmsh.merge("Negativo Gripper v1.step")
gmsh.model.occ.synchronize()
gmsh.model.getEntities(dim=2)
defineMeshSizes(3)
gmsh.model.mesh.generate(2)
# gmsh.model.mesh.refine()
gmsh.write("Finger_cavity.stl")
gmsh.fltk.run()

gmsh.clear()

gmsh.merge("Unión Gripper 1 v2.step")
gmsh.model.occ.synchronize()
defineMeshSizes(2)
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()

gmsh.write("Finger_visu.stl")