function runKF(frictionLevel,varargin)

switch frictionLevel
    case 'High'
    data = loadKFData('Slalom_30kph_HighMu');
    case 'Low'
    data = loadKFData('Slalom_30kph_LowMu');
end

if nargin > 1
    KF = varargin{1};
    Q = KF.Q;
    R = KF.R;
    PInit = KF.PInit;
else
    Q = diag([1e-3 1e-5]);
    R = diag([0.00279 1.96e-5]);
    PInit = diag([1e-9 1e-9]); %Initial covariance
end

vehicleParams = setVehicleParams;
vx = 30/3.6; % m/s
T = 0.001; %KF time step (s)
xInit = [data.truth.aSlipCar(1); data.sensor.nYawMeas(1)]; %Initial state

% Setup state space model
[A, B, C, D] = setupKF(vehicleParams,vx);
sys = ss(A,B,C,D);
sysd = c2d(sys,T);
[A, B, C, D] = ssdata(sysd);

xPred = zeros(2,numel(data.time));
xEst = zeros(2,numel(data.time));
PPred = zeros(2,2,numel(data.time));
PEst = zeros(2,2,numel(data.time));

y = [data.sensor.gLatMeas'; data.sensor.nYawMeas']; %Measurements
u = data.sensor.aWheelFMeas; %Inputs
xEst(:,1) = xInit;
PEst(:,:,1) = PInit; 

%KF prediction and update cycle
for i = 2:numel(data.time)
    [xPred(:,i), PPred(:,:,i)] = kf_predict(xEst(:,i-1),PEst(:,:,i-1),A,Q,B,u(i));
    [xEst(:,i), PEst(:,:,i)] = KFUpdate(xPred(:,i),PPred(:,:,i),y(:,i),u(i),C,D,R);
end

figure
hold on
plot(data.time,data.truth.aSlipCar*180/pi)
plot(data.time,xEst(1,:)*180/pi)
xlabel('Time (s)')
ylabel('Sideslip angle (deg)')
legend('Actual','Estimate')
