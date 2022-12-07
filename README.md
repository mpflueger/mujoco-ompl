# mujoco-ompl

**_This is research code!_** It is provided without any warranty, expressed or implied.  Other researchers are welcome to use this code for academic/non-commercial purposes.  If you are using our code, drop us a line, we would love to know what you are doing and how it is useful to you.

This project is designed as an interface layer between OMPL and MuJoCo.  It builds planning spaces for OMPL based on MuJoCo environment specifications and allows OMPL planners to plan on those spaces by using MuJoCo as the underlying state propagator.  It was originally developed by Max Pflueger as a way of generating plan demonstrations for the [Plan-Space State Embedding](https://github.com/mpflueger/plan-space-state-embedding) project, and has since been extended for use in other ongoing projects.

Some of the code found here is mildly project specific, and in the future we may move it elsewhere to clean up a bit and make this more of a pure library.

**Contact:** Max Pflueger `pflueger` at `usc.edu`


## Build Instructions (new)

Note: below is currently only tested on Ubuntu 20.04.

1. Download MuJoCo release from https://github.com/deepmind/mujoco/releases/tag/2.3.0 (last tested version: 2.3.0)

2. Extract the content of compressed MuJoCo release into a folder `mujoco-2.3.0` under this repo's root directory.

3. Install OMPL following instructions on https://ompl.kavrakilab.org/installation.html (last tested version 1.5.0)

4. Ensure the following libraries can be found by CMake: ompl, Eigen3, GLEW, GLFW, Threads, yaml-cpp, Boost.

5. Build:
   ```
   mkdir build && cd build
   cmake ..
   make
   ```

   Example output of `make`:
   ```
   [ 11%] Built target mujoco_ompl_nogl
   [ 26%] Built target smooth_plan
   [ 42%] Built target render_plan_kinematic
   [ 57%] Built target plan_kinematic
   [ 69%] Built target mujoco_ompl
   Scanning dependencies of target render_plan
   [ 73%] Building CXX object CMakeFiles/render_plan.dir/render_plan.cpp.o
   [ 76%] Building CXX object CMakeFiles/render_plan.dir/src/mujoco_wrapper.cpp.o
   [ 80%] Building CXX object CMakeFiles/render_plan.dir/src/mujoco_ompl_interface.cpp.o
   [ 84%] Linking CXX executable render_plan
   [ 84%] Built target render_plan
   Scanning dependencies of target plan
   [ 88%] Building CXX object CMakeFiles/plan.dir/plan.cpp.o
   [ 92%] Building CXX object CMakeFiles/plan.dir/src/mujoco_wrapper.cpp.o
   [ 96%] Building CXX object CMakeFiles/plan.dir/src/mujoco_ompl_interface.cpp.o
   [100%] Linking CXX executable plan
   [100%] Built target plan
      ```


## Run (new)

Note: below is currently only tested on Ubuntu 20.04.


**To create a plan file** (Assume you are in the repo root directory):
```
./build/plan ./problems/reacher_prob.yaml -o reacher_plan.out
```
The output of this program ends with
```
...
Found Solution with status: Exact solution
Solution wrote to file "reacher_sol.out"
```

**To visualize a plan as a graph** (Assume you are in repo root directory):
```
python3 plot_plan.py reacher_sol.out problems/reacher_info.yaml
```

It may look like this:

<img alt="reacher_plan_graph" src="https://user-images.githubusercontent.com/7720184/205773512-b4c8dce1-c341-45c5-ae98-4d956a513514.png" width="300px"/>



**To rollout a plan in MuJoCo with rendering** (Assume you are in repo root directory)
```
./build/render_plan problems/reacher.xml build/reacher_sol.out
```

You may see the reacher spinning in place, like:

<img alt="reacher_plan_render" src="https://user-images.githubusercontent.com/7720184/205774142-d90aeb2b-15dc-40bb-b4ef-4560028110f3.png" width="400px"/>


[**etpr**](https://github.com/etpr) added kinematic planner and collision check (see [pull request](https://github.com/mpflueger/mujoco-ompl/pull/3)).
You can try it with the 2d_point problem by
```
./build/plan_kinematic problems/2d_point.xml problems/2d_point_prob.yaml 1
```
The plan will be written to `plan.out`. You can render it by
```
./build/render_plan_kinematic problems/2d_point.xml ./plan.out
```
Visualization:

<img alt="render_plan_collision_avoidance" src="https://user-images.githubusercontent.com/7720184/206159014-f62194c1-0a68-4a12-b715-0c74ed6b7022.png" width="400px"/>

