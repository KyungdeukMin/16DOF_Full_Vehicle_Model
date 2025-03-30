function [rm]=angle2rm(roll,pitch,yaw)
cg=cos(roll);
sg=sin(roll);
cb=cos(pitch);
sb=sin(pitch);
ca=cos(yaw);
sa=sin(yaw);

rm=[ca, -sa, 0; sa, ca, 0; 0, 0, 1]*[cb, 0, sb; 0, 1, 0; -sb, 0, cb]*[1, 0, 0; 0, cg, -sg; 0, sg, cg];
return
