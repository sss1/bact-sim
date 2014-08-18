bact-sim
========

Run run_sim to run the simulation.

run_sim sets up the simulation parameters (both inputs and outputs) and then calls basic_swarm with the appropriate inputs and saves its outputs.

Most of the simulation parameters are built into one of two structs: bargs (bacteria arguments) and sargs (simulation arguments).

bargs should ideally be specified using a preset configuration by calling preset(cfg), where cfg is a string specifying the name of the configuration.

sargs should be specified manually, as done in run_sim.m.
