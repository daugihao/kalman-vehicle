function runUKF3and4(frictionLevel,varargin)

%Runs and compares two different UKFs for tracking the motion of a
%passenger car. One uses a constant acceleration model for prediction with measurement updates from inertial sensors and a GPS (UKF3) and the other
%uses forward integration of the inertial sensors as the prediction model
%with updates from a GPS (UKF4).
switch frictionLevel
    case 'High'
        data = loadKFData('Slalom_30kph_HighMu');
    case 'Low'
        data = loadKFData('Slalom_30kph_LowMu');
end

if nargin > 1
    KF = varargin{1};
    Q3 = KF.Q3;
    Q4 = KF.Q4;
    R = KF.R;
    PInit3 = KF.PInit3;
    PInit4 = KF.PInit4;
    tPositionUpdate = KF.dtGPS;
else
    Q3 = diag([1e2 1e-10 1e-10 1e2 1e-10 1e-10 1e2 1e-10 1e-10]); % Process noise covariance matrix
    Q4 = diag([0.00279*0.001^2 1e-10 0.00279*0.001^2 1e-10 1.96e-5*0.001^2]); % Process noise covariance matrix
    R = diag([0.00279 0.00279 1.96e-5 1e-7 1e-7]); % Measurement noise covariance matrix including position updates (elements will be automatically removed where necessary is a subset of measurements is used)
    PInit3 = diag([1e-9 1e-9 1e-9 1e-9 1e-9 1e-9 1e-9 1e-9 1e-9]); % Initial state estimate covariance matrix
    PInit4 = diag([1e-9 1e-9 1e-9 1e-9 1e-9]); % Initial state estimate covariance matrix
    tPositionUpdate = 0.001;
end

RDiag = diag(R);
Ra = diag(RDiag);
Rb = diag([RDiag(1) RDiag(2) RDiag(3)]);
Rc = diag([RDiag(4) RDiag(5)]);

alpha = 1e-3;
beta = 2;
T = 0.001;
xInit3 = [data.sensor.gLongMeas(1); data.truth.vxCar(1); data.sensor.xCarMeas(1);...
    data.sensor.gLatMeas(1); data.truth.vyCar(1); data.sensor.yCarMeas(1);...
    0; data.sensor.nYawMeas(1); data.truth.nYaw(1)];
xInit4 = [data.truth.vxCar(1); data.sensor.xCarMeas(1);...
    data.truth.vyCar(1); data.sensor.yCarMeas(1);...
    data.truth.nYaw(1)];

xPred3 = zeros(9,numel(data.time));
xEst3 = zeros(9,numel(data.time));
PPred3 = zeros(9,9,numel(data.time));
PEst3 = zeros(9,9,numel(data.time));
xPred4 = zeros(5,numel(data.time));
xEst4 = zeros(5,numel(data.time));
PPred4 = zeros(5,5,numel(data.time));
PEst4 = zeros(5,5,numel(data.time));

ya = [data.sensor.gLongMeas'; data.sensor.gLatMeas'; data.sensor.nYawMeas'; data.sensor.xCarMeas'; data.sensor.yCarMeas'];
yb = [data.sensor.gLongMeas'; data.sensor.gLatMeas'; data.sensor.nYawMeas'];
yc = [data.sensor.xCarMeas'; data.sensor.yCarMeas'];
xEst3(:,1) = xInit3;
PEst3(:,:,1) = PInit3;
xEst4(:,1) = xInit4;
PEst4(:,:,1) = PInit4;

u = yb;

for i = 2:numel(data.time)
    [xPred3(:,i), PPred3(:,:,i)] = ukf_predict1(xEst3(:,i-1),PEst3(:,:,i-1),'UKFPredict3',Q3,{T},alpha,beta);
    [xPred4(:,i), PPred4(:,:,i)] = ukf_predict1(xEst4(:,i-1),PEst4(:,:,i-1),'UKFPredict4',Q4,{u(:,i),T},alpha,beta);
    if mod(data.time(i),tPositionUpdate) == 0
        [xEst3(:,i), PEst3(:,:,i)] = ukf_update1(xPred3(:,i),PPred3(:,:,i),ya(:,i),'UKFUpdate3a',Ra,{},alpha,beta);
        [xEst4(:,i), PEst4(:,:,i)] = ukf_update1(xPred4(:,i),PPred4(:,:,i),yc(:,i),'UKFUpdate4',Rc,{},alpha,beta);
    else
        [xEst3(:,i), PEst3(:,:,i)] = ukf_update1(xPred3(:,i),PPred3(:,:,i),yb(:,i),'UKFUpdate3b',Rb,{},alpha,beta);
        xEst4(:,i) = xPred4(:,i);
        PEst4(:,:,i) = PPred4(:,:,i);
    end  
end

figure
hold on
plot(data.time,data.truth.aSlipCar*180/pi)
plot(data.time,atan(xEst3(5,:)./xEst3(2,:))*180/pi)
plot(data.time,atan(xEst4(3,:)./xEst4(1,:))*180/pi)
xlabel('Time (s)')
ylabel('Sideslip angle (deg)')
legend('Actual','Const Accel','Forward Int')