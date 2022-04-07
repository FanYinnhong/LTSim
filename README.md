# LTSim：Light Traffic SIMulation 

## Objectives

Develop a simulation platform for traffic research. 

The LTSim have light renders for the convenience of traffic research.

## Versions

### V0.1 2022-03-09

- Load map from XML files according to [ASAM OpenDRIVE®](https://www.asam.net/standards/detail/opendrive/).
- Generate vehicles randomly and let them run in the road network.


### V0.2 2022-04-07

- Realize the zooming and panning of visual interface to clearly display visual details.
- According to the mouse position, multithreading outputs help text to the display surface in real time.
- Constructing the signal system that controls the passage of vehicles at each entrance of the intersection (To be improved).
- Realize the off-line simulation process: map loading → vehicle flow generation → vehicle control → visualization.
- Based on Frenet coordinate system, constructing perception, prediction and planning module.
- Trajectory fitting for multiple target points and realize the basic car-following and lane-changing processes.
