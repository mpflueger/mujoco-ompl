# mujoco-ompl

## Requirements
#### OMPL
Install from here: https://ompl.kavrakilab.org/installation.html

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
