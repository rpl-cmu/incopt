# INCOPT

This repo contains the source code for the IROS 2022 paper <i>InCOpt: Incremental Constrained Optimization Using the Bayes Tree</i> 

This repo contains all modifications to the core GTSAM library (INCOPT) as well as python scripts/wrappers to run the examples

<b>Note: The instructions below will install the included GTSAM library system wide (replacing any existing installation)</b> 

## ANACONDA ENVIRONMENT 
  Create a virtual python environment using [Anaconda](https://www.anaconda.com/products/individual):
  ```
  conda create -n incopt python=3.7
  conda activate incopt
  ```

## PYTHON PACKAGES
  ```
  pip install pyparsing
  pip install pybind 
  pip install matplotlib
  pip install colorama
  pip install hydra-core --upgrade 
  ```
  (pip version 20.0.2 might be required for pybind: `pip install pip==20.0.2`)


## GTSAM INSTALLATION
  Please make sure to install all of GTSAM prerequisites: https://gtsam.org/get_started/

  From the root directory:
  ```
    cd gtsam 
    mkdir build 
    cd build
    cmake -DGTSAM_BUILD_PYTHON=ON -DGTSAM_PYTHON_VERSION=3.7 -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_WITH_TBB=OFF ..
  ```  

## GTSAM C++ PYTHON WRAPPER 
  From the root directory:
  ```
    cd wrap 
    mkdir build 
    cd build
    cmake ..
    make -j4 
    sudo make install 
  ```
## INSTALLING INCOPT PACKAGES

 From the root directory:
  ``` 
    mkdir build
    cd build
    cmake .. 
  ```
## Install all modules:

From the root directory:
```
  sudo ./rebuild_all.sh
```

## Install incoptpy 
```
cd python 
pip install -e .
```

# Running the python examples 

## 2D Navigation 
The configuration file is located ``python/config/navigation_2D.yaml``

 From the root directory:
  ``` 
    cd python/examples/navigation_2D
    python navigation2D.py
  ```
  
  The plots of the trajectories at each time step will be saved in ``python/figures/navigation_2D/trajectories``

## 2D Planar Pushing
The configuration file is located ``python/config/push_estimation_pybullet.yaml``

 From the root directory:
  ``` 
    cd python/examples/navigation_2D
    python navigation2D.py
  ```
  
  The plots of the trajectories at each time step will be saved in ``python/figures/navigation_2D/trajectories``


## 3D Planning
The configuration file is located ``python/config/arm3_planning_obstacles.yaml``

 From the root directory:
  ``` 
    cd python/examples/path_planning
    python arm3_wam_planning_obstacles.py
  ```
  
  The plots of the planned trajectories at each step and cost_sigma value will be saved in ``python/figures/path_planning/3D/solution``
