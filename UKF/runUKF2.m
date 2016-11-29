function runUKF2(frictionLevel,varargin)

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
    PInit = KF.PInit; %Initial state estimate covariance
    tPositionUpdate = KF.dtGPS; %Sample time (s) for GPS measurements
    tIMUUpdate = KF.dtIMU; %Sample time (s) for IMU measurements
else
    Q = diag([1e-3 1e-5 1e-12 5e5 5e5 1e-12 1e-12]); 
    R = diag([0.00279 1.96e-5 1e-7 1e-7]);
    PInit = diag([1e-9 1e-9 1e-9 1e-9 1e-9 1e-9 1e-9]);
    tPositionUpdate = inf;
    tIMUUpdate = 0.001;
end

%Set 3 different measurement covariances depending on which measurements
%have been received at each time step.
RDiag = diag(R);
Ra = diag(RDiag);
Rb = diag([RDiag(1) RDiag(2)]);
Rc = diag([RDiag(3) RDiag(4)]);

vehicleParams = setVehicleParams;
vx = 30/3.6; % m/s
alpha = 1e-3; %UKF scaling parameter
beta = 2; %UKF shaping parameter
T = 0.001; %UKF time step
xInit = [data.truth.aSlipCar(1); data.sensor.nYawMeas(1); 0; 0; 0; data.sensor.xCarMeas(1); data.sensor.yCarMeas(2)]; %Initial state

xPred = zeros(7,numel(data.time));
xEst = zeros(7,numel(data.time));
PPred = zeros(7,7,numel(data.time));
PEst = zeros(7,7,numel(data.time));

%Set various combinations of measurements possible at each time step
ya = [data.sensor.gLatMeas'; data.sensor.nYawMeas'; data.sensor.xCarMeas'; data.sensor.yCarMeas']; 
yb = [data.sensor.gLatMeas'; data.sensor.nYawMeas'];
yc = [data.sensor.xCarMeas'; data.sensor.yCarMeas'];

u = data.sensor.aWheelFMeas; %Model input
xEst(:,1) = xInit;
PEst(:,:,1) = PInit;

%UKF prediction and update cycle
for i = 2:numel(data.time)
    [xPred(:,i), PPred(:,:,i)] = ukf_predict1(xEst(:,i-1),PEst(:,:,i-1),'UKFPredict2',Q,{u(i),vehicleParams,vx,T},alpha,beta); %Prediction step
    if mod(data.time(i),tPositionUpdate) == 0 && mod(data.time(i),tIMUUpdate) == 0 %Update step if we have all new measurements
        [xEst(:,i), PEst(:,:,i)] = ukf_update1(xPred(:,i),PPred(:,:,i),ya(:,i),'UKFUpdate2a',Ra,{u(i),vehicleParams,vx},alpha,beta);
    elseif mod(data.time(i),tIMUUpdate) == 0 %Update step if we have only new inertial measurements
        [xEst(:,i), PEst(:,:,i)] = ukf_update1(xPred(:,i),PPred(:,:,i),yb(:,i),'UKFUpdate2b',Rb,{u(i),vehicleParams,vx},alpha,beta);
    elseif mod(data.time(i),tPositionUpdate) == 0 %Update step if we have only new GPS measurements
        [xEst(:,i), PEst(:,:,i)] = ukf_update1(xPred(:,i),PPred(:,:,i),yc(:,i),'UKFUpdate2c',Rc,{u(i),vehicleParams,vx},alpha,beta);
    else %Update step if we have no new measurements
        xEst(:,i) = xPred(:,i);
        PEst(:,:,i) = PPred(:,:,i);
    end   
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
f = [vehicleParams.Cf + xEst(4,:)' + 2*sqrt(squeeze(PEst(4,4,:))); flipdim(vehicleParams.Cf + xEst(4,:)' - 2*sqrt(squeeze(PEst(4,4,:))),1)];
fill([data.time; flipdim(data.time,1)],f,[7 7 7]/8)
l1 = plot([data.time(1) data.time(end)],[vehicleParams.Cf vehicleParams.Cf]);
l2 = plot(data.time,vehicleParams.Cf+xEst(4,:));
xlabel('Time (s)')
ylabel('Front axle cornering stiffness (N/rad)')
legend([l1, l2],'Nominal','Estimate')

figure
hold on
f = [vehicleParams.Cr + xEst(5,:)' + 2*sqrt(squeeze(PEst(5,5,:))); flipdim(vehicleParams.Cr + xEst(5,:)' - 2*sqrt(squeeze(PEst(5,5,:))),1)];
fill([data.time; flipdim(data.time,1)],f,[7 7 7]/8)
l1 = plot([data.time(1) data.time(end)],[vehicleParams.Cr vehicleParams.Cr]);
l2 = plot(data.time,vehicleParams.Cr+xEst(5,:));
xlabel('Time (s)')
ylabel('Rear axle cornering stiffness (N/rad)')
legend([l1, l2], 'Nominal','Estimate')
legend('Nominal','Estimate')

