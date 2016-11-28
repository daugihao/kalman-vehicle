function data = loadKFData(filename)

load(filename)

data.time = RunContainers.Run.Data.PersistentResults.Complete.Parameter.Time;
data.sensor.nYawMeas = RunContainers.Run.Data.PersistentResults.Complete.Parameter.nzSensor;
data.sensor.gLatMeas = RunContainers.Run.Data.PersistentResults.Complete.Parameter.gLatSensor;
data.sensor.gLongMeas = RunContainers.Run.Data.PersistentResults.Complete.Parameter.gLongSensor;
data.sensor.aWheelFMeas = RunContainers.Run.Data.PersistentResults.Complete.Parameter.aWheelFMean;
data.truth.aSlipCar = RunContainers.Run.Data.PersistentResults.Complete.Parameter.aSlipCar;
data.truth.vxCar = RunContainers.Run.Data.PersistentResults.Complete.Parameter.vxCar;
data.truth.vyCar = RunContainers.Run.Data.PersistentResults.Complete.Parameter.vyCar;
data.truth.nYaw = RunContainers.Run.Data.PersistentResults.Complete.Parameter.nYaw;

data.sensor.xCarMeas = RunContainers.Run.Data.PersistentResults.Complete.Parameter.xCar;
data.sensor.yCarMeas = RunContainers.Run.Data.PersistentResults.Complete.Parameter.yCar;

end