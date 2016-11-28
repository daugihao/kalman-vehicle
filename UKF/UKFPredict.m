function x = UKFPredict(x,params)
%Bicycle model with linear adaptive tyres

%State vector: [sideslip angle, yaw rate, front cornering stiffness delta,
%rear cornering stiffness delta]

u = params{1};
p = params{2};
vx = params{3};
T = params{4};

Cf = p.Cf + x(3);
Cr = p.Cr + x(4);

A = [-(Cf+Cr)/(p.m*vx)  -(p.a*Cf-p.b*Cr)/(p.m*vx^2)-1 0 0;
    -(p.a*Cf-p.b*Cr)/p.Iz  -(p.a^2*Cf + p.b^2*Cr)/(p.Iz*vx) 0 0;
    0 0 0 0;
    0 0 0 0];
    
B = [Cf/(p.m*vx);
    p.a*Cf/p.Iz;
    0;
    0];

dx = A*x + B*u;
x = x + dx*T;

end