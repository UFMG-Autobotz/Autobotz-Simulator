> This directory contains the 3D mesh used to create models for the Autobotz Simulator.

# Mesh formats

Gazebo can import mesh files of the following formats:

* **.stl:** STL (STereo-Lithography) is the format used on 3D printing, multiple software can export it.

* **.dae:** Collada (COLLAborative Design Activity) are XML base files developed to transfer CAD files from one software to another. They can have textures.

> **Note:**

> When using Collada files with textures, make sure to put the texture on the `materials/texture` directory and change the texture adress inside of the .dae file.
