function xNew = UKFPredict3(xOld,params)

%Constant acceleration particle model

T = params{1};

%State vector: [time derivative of longitudinal velocity, longitudinal velocity, world frame x position
%time derivative of lateral velocity, lateral velocity, world frame y position, 
% yaw acceleration, yaw rate, yaw angle]

xNew = zeros(size(xOld));

xNew([1 4 7]) = xOld([1 4 7]); %Constant acceleration model
    
xNew([2 5 8]) = xOld([2 5 8]) + xOld([1 4 7])*T; %Euler integration acceleration to velocity

xNew(9) = xOld(9) + xOld(8)*T; %Euler integration yaw rate to yaw angle

vy = xOld(5);
vx = xOld(2);
xDot = vx*cos(xOld(9)) - vy*sin(xOld(9));
yDot = vy*cos(xOld(9)) + vx*sin(xOld(9));

%Integration of world frame velocity to world frame position
xNew(3) = xOld(3) + xDot*T; 
xNew(6) = xOld(6) + yDot*T;

end