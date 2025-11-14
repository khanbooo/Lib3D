Realisation of segment intersection in 3D with auto-generating tests.

How to use:
  - clone repository
  - cd to the base directory of cloned repository
  - make
  - make run (to run python tests script)

  In manual execution you should basically provide the program with 12 floating point numbers by the template below:
  - ./build/main.exe s1_x s1_y s1_z e1_x e1_y e1_z s2_x s2_y s2_z e2_x e2_y e2_z
  where:
  s stands for the start vector
  e stands for the end vector
  1 stands for the first segment
  2 stands for the second
  x, y and z are coordinates in 3D

Note:
  - All vectors starts in the origin of coordinates
  - Segments are defined with two vectors: start and end