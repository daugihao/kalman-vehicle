function runUKF(frictionLevel,varargin)

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
    Q = diag([1e-3 1e-5 5e5 5e5]);
    R = diag([0.00279 1.96e-5]);
    PInit = diag([1e-9 1e-9 1e-9 1e-9]); %Initial state estimate covariance
end

vehicleParams = setVehicleParams;
vx = 30/3.6; % m/s
alpha = 1e-3; %Unscented Transform scaling parameter
beta = 2; %Unscented Transform shaping parameter
T = 0.001; %UKF time step (s)
xInit = [data.truth.aSlipCar(1); data.sensor.nYawMeas(1); 0; 0]; %Initial state

xPred = zeros(4,numel(data.time));
xEst = zeros(4,numel(data.time));
PPred = zeros(4,4,numel(data.time));
PEst = zeros(4,4,numel(data.time));

y = [data.sensor.gLatMeas'; data.sensor.nYawMeas']; %Measurements
u = data.sensor.aWheelFMeas; %Inputs
xEst(:,1) = xInit;
PEst(:,:,1) = PInit; 

%UKF prediction and update cycle
for i = 2:numel(data.time)
    [xPred(:,i), PPred(:,:,i)] = ukf_predict1(xEst(:,i-1),PEst(:,:,i-1),'UKFPredict',Q,{u(i),vehicleParams,vx,T},alpha,beta);
    [xEst(:,i), PEst(:,:,i)] = ukf_update1(xPred(:,i),PPred(:,:,i),y(:,i),'UKFUpdate',R,{u(i),vehicleParams,vx},alpha,beta);
end

figure
hold on
plot(data.time,data.truth.aSlipCar*180/pi)
plot(data.time,xEst(1,:)*180/pi)
xlabel('Time (s)')
ylabel('Sideslip angle (deg)')
legend('Actual','Estimate')

figure
hold on
plot([data.time(1) data.time(end)],[vehicleParams.Cf vehicleParams.Cf])
plot(data.time,vehicleParams.Cf+xEst(3,:))
xlabel('Time (s)')
ylabel('Front axle cornering stiffness (N/rad)')
legend('Nominal','Estimate')

figure
hold on
plot([data.time(1) data.time(end)],[vehicleParams.Cr vehicleParams.Cr])
plot(data.time,vehicleParams.Cr+xEst(4,:))
xlabel('Time (s)')
ylabel('Rear axle cornering stiffness (N/rad)')
legend('Nominal','Estimate')
