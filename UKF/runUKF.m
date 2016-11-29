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
f = [xEst(1,:)' + 2*sqrt(squeeze(PEst(1,1,:))); flipdim(xEst(1,:)' - 2*sqrt(squeeze(PEst(1,1,:))),1)];
fill([data.time; flipdim(data.time,1)],f*180/pi,[7 7 7]/8)
l1 = plot(data.time,data.truth.aSlipCar*180/pi);
l2 = plot(data.time,xEst(1,:)*180/pi);
xlabel('Time (s)')
ylabel('Sideslip angle (deg)')
legend([l1, l2],'Actual','Estimate')

figure
hold on
f = [vehicleParams.Cf + xEst(3,:)' + 2*sqrt(squeeze(PEst(3,3,:))); flipdim(vehicleParams.Cf + xEst(3,:)' - 2*sqrt(squeeze(PEst(3,3,:))),1)];
fill([data.time; flipdim(data.time,1)],f,[7 7 7]/8)
l1 = plot([data.time(1) data.time(end)],[vehicleParams.Cf vehicleParams.Cf]);
l2 = plot(data.time,vehicleParams.Cf+xEst(3,:));
xlabel('Time (s)')
ylabel('Front axle cornering stiffness (N/rad)')
legend([l1, l2],'Nominal','Estimate')

figure
hold on
f = [vehicleParams.Cr + xEst(4,:)' + 2*sqrt(squeeze(PEst(4,4,:))); flipdim(vehicleParams.Cr + xEst(4,:)' - 2*sqrt(squeeze(PEst(4,4,:))),1)];
fill([data.time; flipdim(data.time,1)],f,[7 7 7]/8)
l1 = plot([data.time(1) data.time(end)],[vehicleParams.Cr vehicleParams.Cr]);
l2 = plot(data.time,vehicleParams.Cr+xEst(4,:));
xlabel('Time (s)')
ylabel('Rear axle cornering stiffness (N/rad)')
legend([l1, l2], 'Nominal','Estimate')
