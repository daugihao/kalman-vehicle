function y = UKFUpdate3a(x,params)

%Output vector: [longitudinal acceleration, lateral acceleration, yaw rate, x world position, y world position]

y = [x(1) - x(5)*x(8); x(4) + x(2)*x(8); x(8); x(3); x(6)];

end