function [Fxcg,Fycg,Fzcg,Mxcg,Mycg,Mzcg]=Generalized_Force(par,delta,Fxw,Fyw,ls,lw,Fzc,Fxca,rot_matrix)

Fxc(1)=cos(delta(1))*Fxw(1) - sin(delta(1))*Fyw(1);
Fxc(2)=cos(delta(2))*Fxw(2) - sin(delta(2))*Fyw(2);
Fxc(3)=Fxw(3);
Fxc(4)=Fxw(4);

Fyc(1)=cos(delta(1))*Fyw(1) + sin(delta(1))*Fxw(1);
Fyc(2)=cos(delta(2))*Fyw(2) + sin(delta(2))*Fxw(2);
Fyc(3)=Fyw(3);
Fyc(4)=Fyw(4);

Mxc(1)=Fyc(1)*(par.h0-ls(1)-lw(1)) + Fzc(1)*par.b;
Mxc(2)=Fyc(2)*(par.h0-ls(2)-lw(2)) - Fzc(2)*par.b;
Mxc(3)=Fyc(3)*(par.h0-ls(3)-lw(3)) + Fzc(3)*par.b;
Mxc(4)=Fyc(4)*(par.h0-ls(4)-lw(4)) - Fzc(4)*par.b;

Myc(1)=-Fxc(1)*(par.h0-ls(1)-lw(1)) - Fzc(1)*par.af;
Myc(2)=-Fxc(2)*(par.h0-ls(2)-lw(2)) - Fzc(2)*par.af;
Myc(3)=-Fxc(3)*(par.h0-ls(3)-lw(3)) + Fzc(3)*par.ar;
Myc(4)=-Fxc(4)*(par.h0-ls(4)-lw(4)) + Fzc(4)*par.ar;

Mzc(1)=-Fxc(1)*par.b + Fyc(1)*par.af;
Mzc(2)= Fxc(2)*par.b + Fyc(2)*par.af;
Mzc(3)=-Fxc(3)*par.b - Fyc(3)*par.ar;
Mzc(4)= Fxc(4)*par.b - Fyc(4)*par.ar;

%%ac=fc/mc
gravity=par.mc*par.g*rot_matrix^-1*[0;0;-1];

Fxcg=sum(Fxc) + gravity(1)- Fxca;
Fycg=sum(Fyc) + gravity(2);
Fzcg=sum(Fzc) + gravity(3);
Mxcg=sum(Mxc);
Mycg=sum(Myc);
Mzcg=sum(Mzc);


return