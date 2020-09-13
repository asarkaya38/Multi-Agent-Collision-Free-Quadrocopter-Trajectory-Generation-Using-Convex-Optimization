# Multi-Agent-Collision-Free-Quadrocopter-Trajectory-Generation-Using-Convex-Optimization

In this study, I focus on an algorithm for generating
collision-free quadrocopter trajectories. Starting from
some initial position, the quadrcopters are desired to go to the
final position within a specified time under dynamic constraints.
Given initial/final information of position, velocity, and acceleration,
the problem is formulated as a convex optimization problem.
I utilize an iterative approach to estimate the non-convex
constraints replacing them with their linear approximations and
show simulation results for different scenarios.
