#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 13 16:23:25 2024

@author: lab
"""

import gmsh
from defineMeshSizes import defineMeshSizes

gmsh.initialize()

gmsh.merge("Uniaxial v1.step")
gmsh.model.occ.synchronize()
defineMeshSizes(6)
gmsh.model.mesh.generate(3)
# gmsh.model.mesh.refine()
gmsh.write("Uniaxial.vtk")
gmsh.fltk.run()

gmsh.clear()

gmsh.merge("Negativo Uniaxial v1.step")
gmsh.model.occ.synchronize()
# gmsh.model.getEntities(dim=2)
defineMeshSizes(6)
gmsh.model.mesh.generate(2)
# gmsh.model.mesh.refine()
gmsh.write("Uniaxial_cavity.stl")
gmsh.fltk.run()

gmsh.clear()


gmsh.merge("Uniaxial v1.step")
gmsh.model.occ.synchronize()
defineMeshSizes(3)
gmsh.model.mesh.generate(2)

gmsh.write("Uniaxial_visu.stl")

gmsh.clear()