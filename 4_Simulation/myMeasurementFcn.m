function y = myMeasurementFcn(x)
% --- unpack state ---
ub    = x(1);  vb = x(2);  wb = x(3);       
p     = x(4);  q  = x(5);  r  = x(6);


udot = diff(ub);
vdot = diff(vb);
wdot = diff(wb);


y = [udot vdot wdot p q r];
end