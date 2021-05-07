# Bekker_Wong_GPU

Jason Zhou ME 759 Final Project

This is an initial attempt to bring Bekker-Wong Terrain model into GPU

This implementation targets to achieve:
1. An initial GPU implementation of a 3-dimesional flat B-W terrain
2. A analytical wheel-terrain interaction (limited to 1 pair)
3. Visualization output of both the terrain and wheel object

The limitation is stated as below:

1. The wheel shape is limited to a perfect cylinder
2. B-W terrain is limited and can only be initialized as a flat terrain



Abstract: This project is a preliminary design and implementation of the Soil Contact Model (SCM) based on the
Bekker-Wong semi-empirical deformable terrain model. The final goal of the project is to expand the
SCM terrain to GPU parallel computing in Project Chrono. The current stage of implementation has
successfully accomplished the Core Bekker Realization, the main functionality which needs to be
achieved in the SCM Terrain, and a Bulldozing Effect Realization. The current stage of development is
separate from the core code base of Project Chrono, serving as an experimental program to explore
different parallel algorithms which can be used to achieve the same functionalities the CPU version of
the code can achieve. The major limiting factor in the current stage is neighbour-searching algorithm,
which is believed to be hard to parallelize. Future work is needed to come up with an efficient
neighbour-searching algorithm to improve efficiency, and also a final integration to bring the GPU
version of the terrain model into Project Chrono.

Preliminary Simulation Animations:
Without Bulldozing enabled: https://uwmadison.box.com/s/cytkjtqq4pde5bv701bp1twwksuc34n1
With Bulldozing enabled: https://uwmadison.box.com/s/cvu9m0wpn6gez57uge2l4qh7h1n1wsru
