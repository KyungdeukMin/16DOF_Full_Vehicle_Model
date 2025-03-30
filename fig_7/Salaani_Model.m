function [FY, FX, MZ, MX ] = Salaani_Model(Tire,ALPHA, S, GAMMA, FZ)
% ### SAE AXIS Figure 1. ###
% Input: Tire ? ¡°structure for tire parameters
% ALPhA - lateral slip angle in (rad)
% S - longitudinal slip [-1 to 1]
% GAMMA - inclination angle (rad)
% Fz > 0 -Tire normal force in (lbs) ? must be positive
%
% Output: FX - Longitudinal Force (lbs)
% FY - Lateral force (lbs)
% MZ - Tire aligning torque (ft-lbs)
% MX - Tire overturning moment (ft-lbs)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Written by Dr. Mohamed Kamel Salaani for SAE Paper 2007-01-0816 %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
S = min(abs(S),0.99) * sign(S);
FZ2 = FZ * FZ;
%
%%%%% Physical Parameters from Appendix B %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% tire longitudinal stiffness
CS0 = Tire.CS0; Eta = Tire.Eta; Fzx0 = Tire.Fzx0;
% tire lateral stiffness
CA1 = Tire.CA1; CA2 = Tire.CA2; CAm = Tire.CAm; Fzym = Tire.CAFzm;
% inclination angle lateral force stiffness
GAMMA1 = Tire.GAMMA1; GAMMA2 = Tire.GAMMA2;
% aligning moment pneumatic trail
Tz2 = Tire.tz2; Tz1 = Tire.tz1;
% aligning moment constants
m1 = Tire.m1; m0 = Tire.m0;
% sliding force eccentricity
Epsx = Tire.Epsx;
% overturning moment arm
Tx3 = Tire.Tx3; Tx2 = Tire.Tx2; Tx1 = Tire.Tx1;
% road to test friction rate
MURATIO = Tire.MUNOM / Tire.MUNTEST;
% longitudinal peak coefficient of friction
Mux0 = Tire.Mux0;Mux1 = Tire.Mux1; Mux2 = Tire.Mux2;
% Lateral peak coefficient of friction
Muy0 = Tire.Muy0; Muy1 = Tire.Muy1; Muy2 = Tire.Muy2;
% minimum load for valid peak friction values
Fzmu0 = Tire.Fz0;
% longitudinal sliding coefficient of friction
DMUx3 = Tire.KMUx3; DMUx2 = Tire.KMUx2; DMUx1 = Tire.KMUx1; Eps_sx = Tire.Lsx;
% lateral sliding coefficient of friction
DMUy3 = Tire.KMUy3; DMUy2 = Tire.KMUy2; DMUy1 = Tire.KMUy1; Eps_sy = Tire.Lsy;
%
%%%%%% Empirical physical properties formulae %%%%%%%%%%%%%%%%%%%%%
%
% pneumatic trail (Equation 29)
Tz = Tz1*FZ2 + Tz2*FZ;
% overturning moment arm (Equation 30)
Tx = Tx1*FZ2 + Tx2*FZ + Tx3;
% lateral stiffness (Equation 26)
CA = CAm*(1-exp(CA1*(FZ/Fzym).^2 + CA2*(FZ/Fzym)));
% CA = CAm*(1-exp(CA1*log(FZ/Fzym).^2 + CA2*log(FZ/Fzym)));%130709 min
% longitudinal stiffness (Equation 27)
CS = CS0*(FZ/Fzx0).^Eta;
% Inclination angle lateral force stiffness (Equation 31)
% % % % % % THIS DOCUMENT IS PROTECTED BY U.S. AND INTERNATIONAL COPYRIGHT.
% % % % % % It may not be reproduced, stored in a retrieval system, distributed or transmitted, in whole or in part, in any form or by any means.
% % % % % % Downloaded from SAE International by Kyungdeuk Min, Thursday, June 27, 2013 05:11:07 AM
FYGAMMA = GAMMA1 * FZ + GAMMA2 * FZ2;
% longitudinal peak coefficient of friction (Equation 22)
if FZ < Fzmu0
FZl = Fzmu0;
else
FZl = FZ;
end
MUXp=(MURATIO)*Mux0*(FZl/Fzmu0).^(Mux2+Mux1*log(FZl/Fzmu0));
% lateral peak coefficient of friction (Equation 22)
MUYp=(MURATIO)*Muy0*(FZl/Fzmu0).^(Muy2+Muy1*log(FZl/Fzmu0));
% longitudinal decay of friction (Equation 23)
DMUx = DMUx1*FZ2 + DMUx2*FZ + DMUx3;

% lateral decay of friction (Equation 23)
DMUy = DMUy1*FZ2 + DMUy2*FZ + DMUy3;

% adjust for plysteer
% ALPHA = ALPHA - Tire.PLYSTEER;
% sliding coefficient of friction (Equation 24)
Slip = min(1,sqrt((sin(ALPHA))^2+(S*cos(ALPHA))^2));
MNUy = MUYp * (1 - DMUy * Slip)*Eps_sy;
MNUx = MUXp * (1 - DMUx * Slip)*Eps_sx;
% effect of slip on longitudinal stiffness  (Equation 12)%130709 min
CSp = CS + (CA - CS)*Slip;


%
%%%%%%%%%%%%%%%%%%% Salaani¡¯s Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% adhesion potential rate (Equation 11)
%
SIGMA = sqrt((CA*tan(ALPHA)/(1-S)/MUYp/FZ)^2+(CS*S/(1-S)/MUXp/FZ)^2);
%
% adhesion and sliding functions (Equations 18 and 19)
%
SIGMA2 = SIGMA*SIGMA;
SIGMAU = (1 - SIGMA2)/(1 + SIGMA2);
F_a = 4/pi*SIGMA/(SIGMA2 + 1)^2;
F_s = 1/pi*(pi/2 - SIGMAU*sqrt(1-SIGMAU^2) - asin(SIGMAU));
% F_s = 1/pi*(pi/2 - SIGMAU*2*SIGMA/(1+SIGMA2) - asin(SIGMAU));
%
% lateral and longitudinal forces (Equations 14 and 15)
%
SIGMAm = sqrt( (CA * tan(ALPHA)/MUYp)^2 + (CS * S/MUXp)^2 );
SIGMASm = sqrt( (CA * tan(ALPHA)/MNUy)^2 + (CSp * S/MNUx)^2 );
% (CA * tan(ALPHA)/MNUy)

if (SIGMAm < 1.0e-6) 
FY = 0;
FX = 0;
else
FY = FZ * CA*tan(ALPHA) *( F_a/SIGMAm + F_s/SIGMASm);  
FX = - FZ * S *(CS*F_a/SIGMAm + CSp*F_s/SIGMASm); 

end
%
% aligning torque (Equation 20)
%
MZ = Tz*CA*tan(ALPHA)/(m1*SIGMA2 + m0)^2 + FY*Epsx*F_s;
%
% overturning moment (Equation 21)
%
MX = -Tx*FY;
%
% Camber effect added to lateral force (Equation 32)
%
FY = FY + FYGAMMA * GAMMA * (1 - F_s);