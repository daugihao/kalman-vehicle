function y = UKFUpdate(x,params)

%Output vector: [lateral acceleration, yaw rate]

u = params{1};
p = params{2};
vx = params{3};

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

V = [vx 0 0 0;
    0 0 0 0];

W = [0 vx 0 0;
    0 1 0 0];

C = V*A + W;
D = V*B;

y = C*x + D*u;

end