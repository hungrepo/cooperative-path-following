# Description

This repo contains the code that generates results in the paper:

Nguyen T. Hung, Antonio M. Pascoal, Tor A. Johansen, “Cooperative path following of constrained autonomous vehicles with model predictive control and event-triggered communications”, International Journal of Robust Nonlinear Control, 2020.

See also the detail of the work here: https://nt-hung.github.io/research/cooperative-path-following/

# Installation

**Prerequisite:** [Casadi](https://web.casadi.org/get/) for matlab (the tool to formulate optimal control problem) 

- Clone/Download [Casadi](https://web.casadi.org/get/) from  https://web.casadi.org/get
- Create a folder inside the Casadi package that you just downloaded.
- Download the codes from this repo to the folder just created. 

**Test the codes:**

For triangular formation,  
- run MPC_CPF_5V_Triangular.m

For circular formation
- run MPC_CPF_5V_Circular.m

To plot the results
- run plotjournal_5vehicle.m

**Note**:  

- For every interaction it solves 5 optimization problems (in 5 MPC controllers for the 5 vehicles) so it might take a while to complete the simulation (but just in 1,2 minutes)
- We can simplify the MPC scheme by inactivating the contractive constraint. This can be done in the code, for example, in MPC_CPF_5V_Triangular.m, we can comment those lines in picture "InactivateContractiveConstraint.png". This will reduce the cost of computation and simulation time. But theoretically, stability is not guaranteed as long as the prediction horizon is chosen sufficiently long.
# Citation

If you use the code for your work and publication, please cite the paper with 

Hung, NT, Pascoal, AM, Johansen, TA. Cooperative path following of constrained autonomous vehicles with model predictive control and event‐triggered communications. Int J Robust Nonlinear Control. 2020; 30: 2644– 2670. https://doi.org/10.1002/rnc.4896
