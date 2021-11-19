# ofxLibigl
### libigl - A simple C++ geometry processing library
Will support both Libigl-legacy and Libigl as we go further
Currently using d4b67260855fe901541f2949ff21819147c08a4d tree on master branch

![](https://github.com/libigl/libigl/raw/5ff6387765fa85ca46f1a6222728e35e2b8b8961/libigl-teaser.png)

libigl is a simple C++ geometry processing library. We have a wide
functionality including construction of sparse discrete differential geometry
operators and finite-elements matrices such as the cotangent Laplacian and
diagonalized mass matrix, simple facet and edge-based topology data structures,
mesh-viewing utilities for OpenGL and GLSL, and many core functions for matrix
manipulation which make [Eigen](http://eigen.tuxfamily.org) feel a lot more
like MATLAB.

It is **a header-only library**. You do not need to compile anything to use,
just include igl headers (e.g. `#include <igl/cotmatrix.h>`) and run.  Each
header file contains a single function (e.g. `igl/cotmatrix.h` contains
`igl::cotmatrix()`). Most are tailored to operate on a generic triangle mesh
stored in an n-by-3 matrix of vertex positions `V` and an m-by-3 matrix of
triangle indices `F`.

## Dependencies
Dependencies are on a per-include basis and the majority of the functions in
libigl depends only on the [Eigen](http://eigen.tuxfamily.org) library.

For more information see our [tutorial](http://libigl.github.io/libigl/tutorial).

### Optional Dependencies

Libigl compartmentalizes its **optional** dependences via its directory
organization in the `include/` folder. All header files located _directly_ in
the `include/igl/` folder have only stl and Eigen as dependencies. For example,
all of the headers that depend on CGAL are located in `include/igl/copyleft/cgal`.
For a full list of _optional_ dependencies check `optional/CMakeLists.txt`.

## License
libigl is primarily [MPL2](http://www.mozilla.org/MPL/2.0/) licensed
([FAQ](http://www.mozilla.org/MPL/2.0/FAQ.html)). Some files contain
third-party code under other licenses. We're currently in the processes of
identifying these and marking appropriately.

## Attribution
If you use libigl in your academic projects, please cite the papers we
implement as appropriate. To cite the library in general, you could use this
BibTeX entry:

```bibtex
@misc{libigl,
  title = {{libigl}: A simple {C++} geometry processing library},
  author = {Alec Jacobson and Daniele Panozzo and others},
  note = {http://libigl.github.io/libigl/},
  year = {2017},
}
```

## Copyright
2017 Alec Jacobson, Daniele Panozzo, Christian Schüller, Olga Diamanti, Qingnan Zhou, Sebastian Koch, Jeremie Dumas, Amir Vaxman, Nico Pietroni, Stefan Brugger, Kenshi Takayama, Wenzel Jakob, Nikolas De Giorgis, Luigi Rocca, Leonardo Sacht, Kevin Walliman, Olga Sorkine-Hornung, and others.

Please see individual files for appropriate copyright notices.