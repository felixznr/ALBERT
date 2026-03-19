function x = myStateTransitionFcn(x,u_in)
% x = [u v w p q r phi theta psi]'
% u_in = [delta_eta delta_zeta delta_r T]' 

% Define Parameters

Ts = 0.01;
% --- Geometry / environment ---
g0   = 9.81;        % m/s^2
rho0 = 1.225;       % kg/m^3 (sea level)
S    = 0.010;       % m^2  reference area
d    = 0.10;        % m    reference diameter

% --- Mass / inertia ---
m  = 1.20;          % kg
Ix = 0.005;         % kg*m^2
Iy = 0.020;         % kg*m^2 
Iz = 0.020;         % kg*m^2

% --- Aerodynamic force coeffs ---
CA0       = 0.09;   % axial (drag-like)
CA_kalpha = 0.041;   % axial vs alpha
CA_kbeta  = 0.041;   % axial vs beta


% --- Moment coeffs ---
Cl_delta = 0.229;    % roll control effectiveness
Clp      = -10;     % roll damping
Cm_alpha = 2.0;    % pitch stability derivative
Cm_delta = 1.432;    % pitch control effectiveness
Cmq      = -10;     % pitch damping
Cn_beta  = 2.0;     % yaw stability derivative
Cn_delta = 1.432;     % yaw control effectiveness
Cnr      = -10;     % yaw damping

CN_alpha = -10;
CN_delta = -0.1;

% --- Reference points ---
xcm  = 0.50;        % m 
xref = 0.45;        % m 

% --- unpack state ---
ub    = x(1);  vb = x(2);  wb = x(3);       
p     = x(4);  q  = x(5);  r  = x(6);
phi   = x(7);  theta = x(8); psi = x(9);

% --- unpack inputs ---
delta_eta  = u_in(1);
delta_zeta = u_in(2);
delta_r    = u_in(3);


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

% --- moment coefficients  ---
Cl = Cl_delta*delta_r + (d/(2*V))*Clp*p;

% Pitch & yaw moment coefficients
Cmref = Cm_alpha*alpha + Cm_delta*delta_eta;
Cnref = Cn_beta *beta  + Cn_delta*delta_zeta;

Cm = Cmref - CNz*((xcm-xref)/d) + (d/(2*V))*(Cmq + Cm_alpha)*q;
Cn = Cnref - CNy*((xcm-xref)/d) + (d/(2*V))*(Cnr + Cn_beta )*r;

LA = Qdyn * Cl * S * d;
MA = Qdyn * Cm * S * d;
NA = Qdyn * Cn * S * d;

% Propulsion moments 
Lp = 0; Mp = 0; Np = 0;

% --- translational dynamics in body ---
udot = Fx/m - (q*wb - r*vb);
vdot = Fy/m - (r*ub - p*wb);
wdot = Fz/m - (p*vb - q*ub);

% --- rotational dynamics ---
pdot = (LA + Lp - q*r*(Iz - Iy))/Ix;
qdot = (MA + Mp - r*p*(Ix - Iz))/Iy;
rdot = (NA + Np - p*q*(Iy - Ix))/Iz;

% --- Euler kinematics ---
phidot   = p + (q*sin(phi) + r*cos(phi))*tan(theta);
thetadot = q*cos(phi) - r*sin(phi);
psidot   = (q*sin(phi) + r*cos(phi))/cos(theta);

ub = ub + udot*Ts;
vb = vb + vdot*Ts;
wb = wb + wdot*Ts;

p = p + pdot*Ts;
q = q + qdot*Ts;
r = r + rdot*Ts;

phi = phi + phidot*Ts;
theta = theta + thetadot*Ts;
psi = psi + psidot*Ts;
x = [ub vb wb p q r phi theta psi];
end