# Description

This repo contains the code that generates results in the paper:

Nguyen T. Hung, Antonio M. Pascoal, Tor A. Johansen, “Cooperative path following of constrained autonomous vehicles with model predictive control and event-triggered communications”, International Journal of Robust Nonlinear Control, 2020.

See also the detail of the work here: https://nt-hung.github.io/research/cooperative-path-following/

# Installation

Prerequisite: [Casadi](https://web.casadi.org/get/) for matlab (the tool to formulate optimal control problem) 

For triangular formation,  
- run MPC_CPF_5V_Strategy_I_Event_Straghtline_Delay.m

For circular formation
- run MPC_CPF_5V_Strategy_I_Event_Circle_Delay.m

To plot the result
- run plotjournal_5vehicle.m

**Note**: Because for every interaction, it solves 5 optimization problems (in 5 MPC controllers for 5 vehicles) so it takes a while to complete the simulation (but about 3 minutes for one)

# Citation

If you use the code for your work and publication, please cite the paper with 

Hung, NT, Pascoal, AM, Johansen, TA. Cooperative path following of constrained autonomous vehicles with model predictive control and event‐triggered communications. Int J Robust Nonlinear Control. 2020; 30: 2644– 2670. https://doi.org/10.1002/rnc.4896