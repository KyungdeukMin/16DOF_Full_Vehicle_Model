function [axo,ayo,azo,arxc,aryc,arzc]=Acceleration(par,Fxcg,Fycg,Fzcg,Mxcg,Mycg,Mzcg,vrxc,vryc,vrzc,rot_matrix)

force_abs=rot_matrix*[Fxcg; Fycg; Fzcg];
axo= force_abs(1)/par.mc;
ayo= force_abs(2)/par.mc;
azo= force_abs(3)/par.mc;

% Euler's eq. of motion of a rigid body
arxc= (Mxcg+(par.iyc-par.izc)*vryc*vrzc)/par.ixc;
aryc= (Mycg+(par.izc-par.ixc)*vrzc*vrxc)/par.iyc;
arzc= (Mzcg+(par.ixc-par.iyc)*vrxc*vryc)/par.izc;

return