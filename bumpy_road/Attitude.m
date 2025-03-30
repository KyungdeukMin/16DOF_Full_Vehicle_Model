function [rot_matrix,roll,pitch,yaw]=Attitude(par,vrxc,vryc,vrzc,last_rot_matrix)

rot_instant=angle2rm(vrxc*par.T,vryc*par.T,vrzc*par.T);

rot_matrix=last_rot_matrix*rot_instant;
% data.rot_matrix(i,1:9)=[rot_matrix(1,:),rot_matrix(2,:),rot_matrix(3,:)];

pitch=atan2(-rot_matrix(3,1),(rot_matrix(1,1)^2+rot_matrix(2,1)^2)^(1/2));
yaw=atan2(rot_matrix(2,1)/cos(pitch), rot_matrix(1,1)/cos(pitch));
roll=atan2(rot_matrix(3,2)/cos(pitch),rot_matrix(3,3)/cos(pitch));

return 