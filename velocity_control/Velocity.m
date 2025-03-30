function [vxo,vyo,vzo,vxc,vyc,vzc,vrxc,vryc,vrzc]=Velocity(par,axo,ayo,azo,vxo,vyo,vzo,arxc,aryc,arzc,vrxc,vryc,vrzc,rot_matrix)

vxo=vxo+axo*par.T;
vyo=vyo+ayo*par.T;
vzo=vzo+azo*par.T;

vrxc=vrxc+arxc*par.T;
vryc=vryc+aryc*par.T;
vrzc=vrzc+arzc*par.T;

vel_vec=(rot_matrix^-1*[vxo;vyo;vzo])';

vxc=vel_vec(1);
vyc=vel_vec(2);
vzc=vel_vec(3);

return