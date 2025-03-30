function [out]=Ackerman_Turning_Geometry(par,delta)

if delta==0
delta_l=0;
delta_r=0;
else
Rr=(par.af+par.ar)/tan(delta);
Ri=((Rr-par.b)^2+(par.af+par.ar)^2)^(1/2);
Ro=((Rr+par.b)^2+(par.af+par.ar)^2)^(1/2);
delta_i=acos((Rr-par.b)/Ri);
delta_o=acos((Rr+par.b)/Ro);
delta_l=delta_i;
delta_r=delta_o;
end

if delta_l>(pi/2)
    delta_l=delta_l-pi;
end
if delta_r>(pi/2)
    delta_r=delta_r-pi;
end
out=[delta_l, delta_r];
return