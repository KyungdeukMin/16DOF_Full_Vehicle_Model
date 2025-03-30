function [Fxca]=Aerodynamic_Model(vxc)
rho=1.204; %[kg/m^3]
Cxc=0.3;
Axc=1.820*1.655*0.80; %[m^2]
Fxca=1/2*rho*Cxc*Axc*vxc^2;
return