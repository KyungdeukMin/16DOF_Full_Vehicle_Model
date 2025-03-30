function [out_hw,out_Fzc,out_vhw,out_Fzt,out_Fzw,out_ahw,out_hc,out_ls,out_vls]=Suspension_Model(par, hr, zo, rot_matrix)

persistent hw
persistent Fzc
persistent vhw
persistent last_ls
if isempty(hw)
hw=[0 0 0 0]; %[m]
Fzc=[0 0 0 0]; %[N]
vhw=[0 0 0 0]; %[m/s]
last_ls=[0 0 0 0]; %[m]
end

mw=[50 50 50 50]; %[Kg]
kw=[220000, 220000, 220000, 220000]; %[N/m]
ks=[20000, 20000, 20000, 20000]; %[N/m]
ds=[3000, 3000, 3000, 3000]; %[N/m]

% hw=[0 0 0 0]; %[m]
% Fzc=[0 0 0 0]; %[N]
% vhw=[0 0 0 0]; %[m/s]
% last_ls=[0 0 0 0]; %[m]

% Veh.Fzt=[0 0 0 0]; %[N]
% Veh.Fzw=[0 0 0 0]; %[N]
% Veh.ahw=[0 0 0 0]; %[m/s^2]
% Veh.hc=[0 0 0 0]; %[m]
% Veh.ls=[0 0 0 0]; %[m]
% Veh.vls=[0 0 0 0]; %[m/s]

lw=hr-hw;
if lw(1)<0
    lw(1)=0;
end
if lw(2)<0
    lw(2)=0;
end
if lw(3)<0
    lw(3)=0;
end
if lw(4)<0
    lw(4)=0;
end

Fzt=lw.*kw;

Fzgw=mw*par.g*([0 0 1]*rot_matrix^-1*[0;0;-1]);

Fzw=Fzt - Fzc + Fzgw;
ahw= Fzw./mw;
vhw=vhw+ahw*par.T;
hw=hw+vhw*par.T;

hc(1)=zo + rot_matrix(3,:)*[par.af;par.b;0];
hc(2)=zo + rot_matrix(3,:)*[par.af;-par.b;0];
hc(3)=zo + rot_matrix(3,:)*[-par.ar;par.b;0];
hc(4)=zo + rot_matrix(3,:)*[-par.ar;-par.b;0];

ls=hw-hc;
vls=(ls-last_ls)/par.T;
last_ls=ls;
Fzc=ls.*ks + vls.*ds;

out_hw=hw;out_Fzc=Fzc;out_vhw=vhw;out_Fzt=Fzt;out_Fzw=Fzw;out_ahw=ahw;out_hc=hc;out_ls=ls;out_vls=vls;

return
