function xNew = UKFPredict2(xOld,params)
%Bicycle model with linear adaptive tyres and integration of states up to
%world x-y position

%State vector: [sideslip angle, yaw rate, yaw angle, front cornering
%stiffness delta, rear cornering stiffness delta, world x position, world y
%position]

u = params{1};
p = params{2};
vx = params{3};
T = params{4};

xNew = zeros(size(xOld));

Cf = p.Cf + xOld(4);
Cr = p.Cr + xOld(5);

A = [-(Cf+Cr)/(p.m*vx)  -(p.a*Cf-p.b*Cr)/(p.m*vx^2)-1 0 0 0;
    -(p.a*Cf-p.b*Cr)/p.Iz  -(p.a^2*Cf + p.b^2*Cr)/(p.Iz*vx) 0 0 0;
    0 1 0 0 0;
    0 0 0 0 0;
    0 0 0 0 0];
    
B = [Cf/(p.m*vx);
    p.a*Cf/p.Iz;
    0;
    0;
    0];

vy = tan(xOld(1))*vx;
xDot = vx*cos(xOld(3)) - vy*sin(xOld(3));
yDot = vy*cos(xOld(3)) + vx*sin(xOld(3));

dx = [A*xOld(1:5) + B*u; xDot; yDot];
xNew = xOld + dx*T;

end