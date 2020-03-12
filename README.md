# mujoco-ompl

**_This is research code!_** It is provided without any warranty, expressed or implied.  Other researcher are welcome to use this code for academic/non-commercial purposes.  If you are using our code, drop us a line, we would love to know what you are doing and how it is useful to you.

This project is designed as an interface layer between OMPL and MuJoCo.  It builds planning spaces for OMPL based on MuJoCo environment specifications and allows OMPL planners to plan on those spaces by using MuJoCo as the underlying state propagator.  It was originally developed by Max Pflueger as a way of generating plan demonstrations for the [Plan-Space State Embedding](https://github.com/mpflueger/plan-space-state-embedding) project, and has since been extended for use in other ongoing projects.

Some of the code found here is mildly project specific, and in the future we may move it elsewhere to clean up a bit and make this more of a pure library.

**Contact:** Max Pflueger `pflueger` at `usc.edu`

## Requirements
#### OMPL
Install from here: https://ompl.kavrakilab.org/installation.html
(If you install it from source, make sure to run 'make install'.)

#### MuJoCo
- Download mujoco200 here: https://www.roboti.us/index.html
- Unpack libraries to ~/.mujoco/mujoco200
- Place license file at ~/.mujoco/mjkey.txt

#### Eigen3
#### OpenGL
#### GLEW


## Build
```
cmake .
make
```

## Run
MacOS requires to include the mujoco bin directory in the DYLD_LIBRARY_PATH:
```
DYLD_LIBRARY_PATH=/Users/$USER/.mujoco/mujoco200/bin
```

To create a plan file:
```
./plan reacher.xml reacher_prob.yaml 10
```

To visualize a plan as a graph:
```
python3 plot_plan.py plan.out reacher_info.yaml
```

To rollout a plan in MuJoCo with OpenGL rendering:
```
./render_plan reacher.xml plan.out
```
