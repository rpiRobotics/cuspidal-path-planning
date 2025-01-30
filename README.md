# Cuspidal-Path-Planning

Cuspidal robot identification, path planning, and path optimization from "Path Planning and Optimization for Cuspidal 6R Manipulators."

Robot examples used:
* Cuspidal 3R spatial manipulator
* FANUC CRX-10ia/L
* ABB GoFa

Abstract: A cuspidal robot can move from one inverse kinematics (IK) solution to another without crossing a singularity. Multiple industrial robots are cuspidal. They tend to have a beautiful mechanical design, but they pose path planning challenges.
A task-space path may have a valid IK solution for each point along the path, but a continuous joint-space path may depend on the choice of the IK solution or even be infeasible.
This paper presents new analysis, path planning, and optimization methods to enhance the utility of cuspidal robots. We first demonstrate an efficient method to identify cuspidal robots and show, for the first time, that the ABB GoFa and certain robots with three parallel joint axes are cuspidal.
We then propose a new path planning method for cuspidal robots by finding all IK solutions for each point along a task-space path and constructing a graph to connect each vertex corresponding to an IK solution. Graph edges are weighted based on the optimization metric, such as minimizing joint velocity.
The optimal feasible path is the shortest path in the graph.
This method can find non-singular paths as well as smooth paths which pass through singularities.
Finally, this path planning method is incorporated into a path optimization algorithm. Given a fixed workspace toolpath, we optimize the offset of the toolpath in the robot base frame while ensuring continuous joint motion.
Code examples are available in a publicly accessible repository.


For 6-DOF IK solutions see [ik-geo](https://github.com/rpiRobotics/ik-geo). **Code in this repo depends on that code.**

This repo also depends on [matlab-diagrams](https://github.com/aelias36/matlab-diagrams) for plotting.
