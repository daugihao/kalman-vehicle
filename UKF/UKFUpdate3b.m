function y = UKFUpdate3b(x,params)

%Output vector: [longitudinal acceleration, lateral acceleration, yaw rate]

y = [x(1) - x(5)*x(8); x(4) + x(2)*x(8); x(8)];

end