function [A, B, C, D] = setupKF(p,vx)

%State vector: [sideslip angle, yaw rate]
%Output vector: [lateral acceleration, yaw rate]

A = [-(p.Cf+p.Cr)/(p.m*vx)  -(p.a*p.Cf-p.b*p.Cr)/(p.m*vx^2)-1;
    -(p.a*p.Cf-p.b*p.Cr)/p.Iz  -(p.a^2*p.Cf + p.b^2*p.Cr)/(p.Iz*vx)];
    
B = [p.Cf/(p.m*vx);
    p.a*p.Cf/p.Iz];

V = [vx 0;
    0 0];

W = [0 vx;
    0 1];

C = V*A + W;
D = V*B;
end