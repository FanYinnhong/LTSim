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

### V0.3 2022-05-05

- Adding toolbar which supports suspend, restart and restore functions.
- Adding 'Focus Mode' which offers a static view for observing traffic condition of the selected junction.
- When 'Focus Mode' is on,  a translucent mask is added to shield irrelevant information.
- Adding display area for showing junction condition,  global traffic information, simulation time and etc.
- Fixing bug during zooming and panning of visual interface.
- Supporting information export of the current simulation.
- Perceptiving surrounding vehicles by the recursive judgment of adjacent sections under the Frenet coordinate system.
- Optimize the interactions among vehicles based on the risk perception.
- Adding the constraint of signal time assignment system on vehicle movement within intersections.
