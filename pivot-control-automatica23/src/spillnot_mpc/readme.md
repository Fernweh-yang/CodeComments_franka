The joint velocity controller `JointMPC` defined in `include/spillnot_mpc/spillnot_mpc_j2.h` and `src/spillnot_mpc/spillnot_mpc_j2.cpp` includes and makes use of the class `SpillnotMPC` defined in `include/spillnot_mpc/mpc.cpp` that contains the optimization problem definition and interface to the OSQP solver.

At each control step, the optimization constraints are updated with the latest state estimate, and the updated optimization problem is solved to obtain the desired control command.