function xNew = UKFPredict4(xOld,params)

%Forward integration particle model

u = params{1};
T = params{2};

%State vector: [longitudinal velocity, world frame x position
%lateral velocity, world frame y position, 
%yaw angle]

%Input vector: [longitudinal acceleration, lateral acceleration, yaw rate]

xNew = zeros(size(xOld));

%Calculate time derivatives of longitudinal and lateral velocities
vxDot = u(1) + u(3)*xOld(3);
vyDot = u(2) - u(3)*xOld(1);

%Euler integration of car frame velocities and yaw angle
xNew([1 3 5]) = xOld([1 3 5]) + [vxDot; vyDot; u(3)]*T;

%Calculate world frame velocities
vy = xOld(3);
vx = xOld(1);
xDot = vx*cos(xOld(5)) - vy*sin(xOld(5));
yDot = vy*cos(xOld(5)) + vx*sin(xOld(5));

%Integration of world frame velocity to world frame position
xNew(2) = xOld(2) + xDot*T; 
xNew(4) = xOld(4) + yDot*T;

end