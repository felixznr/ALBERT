function y = myMeasurementFcn(x)


% --- unpack state ---
ub    = x(1);  vb = x(2);  wb = x(3);       
p     = x(4);  q  = x(5);  r  = x(6);
phi   = x(7);  theta = x(8); psi = x(9);

% --- derived quantities ---
V = sqrt(ub^2 + vb^2 + wb^2) + 1e-9;  % avoid div0
alpha = atan2(wb,ub);
beta  = asin(vb / V); 
alpha_total = atan2(sqrt(vb^2 + wb^2), ub);
% Dynamic pressure q = 1/2 p V^2 for moments
Qdyn = 0.5*rho0*V^2;


CA = CA0 + CA_kalpha*alpha + CA_kbeta*beta;
CN = CN_alpha * alpha_total; %+ CN_delta * delta_eff;
% Build direct force coefficiants
CNy = CN * vb / sqrt(vb^2 + wb^2 + 1e-9);
CNz = CN * wb / sqrt(vb^2 + wb^2 + 1e-9);

% Aerodynamic forces 
FAxb = -Qdyn * CA  * S;
FAyb =  Qdyn * CNy * S;
FAzb =  Qdyn * CNz * S;

% --- thrust in body ---
FPxb = 0;  FPyb = 0;  FPzb = 0;

% --- gravity in body: Fg_b = R_nb' * [0;0;mg] in NED ---
% NED: gravity in nav frame is [0;0; +m*g] (Down positive)
Fg_n = [-m*g0; 0;0];

% Rotation matrix R_b^n (ZYX) and R_n^b = (R_b^n)'
cphi = cos(phi); sphi = sin(phi);
cth  = cos(theta); sth = sin(theta);
cps  = cos(psi); sps  = sin(psi);

% ZYX: body->nav (matches common aerospace convention)
R_b_n = [ cth*cps,  sphi*sth*cps - cphi*sps,  cphi*sth*cps + sphi*sps;
          cth*sps,  sphi*sth*sps + cphi*cps,  cphi*sth*sps - sphi*cps;
          -sth,     sphi*cth,              cphi*cth ];
R_n_b = R_b_n';

Fg_b = R_n_b * Fg_n;
Fgxb = Fg_b(1); Fgyb = Fg_b(2); Fgzb = Fg_b(3);

% --- total forces ---
Fx = FAxb + FPxb + Fgxb;
Fy = FAyb + FPyb + Fgyb;
Fz = FAzb + FPzb + Fgzb;



% --- translational dynamics in body ---
udot = Fx/m - (q*wb - r*vb);
vdot = Fy/m - (r*ub - p*wb);
wdot = Fz/m - (p*vb - q*ub);

y = [ub vb wb p q r phi];
end