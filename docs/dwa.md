
- [DWA](http://wiki.ros.org/dwa_local_planner), 1997
    - Summary
        - This package provides an implementation of the Dynamic Window Approach to local robot navigation on a plane.
        - Given a global plan to follow and a costmap, the local planner produces velocity commands to send to a mobile base.
        - This package supports any robot who's footprint can be represented as a convex polygon or circle, and exposes its configuration as ROS parameters that can be set in a launch file.
        - The parameters for this planner are also dynamically reconfigurable.
        - This package's ROS wrapper adheres to the `BaseLocalPlanner` interface specified in the `nav_core` package.
    - 1. Overview

        The `dwa_local_planner` package provides a controller that drives a mobile base in the plane. This controller serves to connect the path planner to the robot. 

        Using a map, the planner creates a kinematic trajectory for the robot to get from a start to a goal location. 

        Along the way, the planner creates, at least locally around the robot, a value function, represented as a grid map. 

        This value function encodes the costs of traversing through the grid cells. 

        The controller's job is to use this value function to determine dx,dy,dtheta velocities to send to the robot.

        ![Local%20planner%201d0461139f5e4eebbe992cc8eea2fc76/Untitled%203.png](../assets/img/Untitled%203.png)

        The basic idea of the Dynamic Window Approach (DWA) algorithm is as follows:

        1. Discretely sample in the robot's control space (dx,dy,dtheta)
        2. For each sampled velocity, perform forward simulation from the robot's current state to predict what would happen if the sampled velocity were applied for some (short) period of time.
        3. Evaluate (score) each trajectory resulting from the forward simulation, using a metric that incorporates characteristics such as: proximity to obstacles, proximity to the goal, proximity to the global path, and speed. Discard illegal trajectories (those that collide with obstacles).
        4. Pick the highest-scoring trajectory and send the associated velocity to the mobile base.
        5. Rinse and repeat.
