clear all
close all

gpsfile = 'NMEA001.csv';
sensorfile = 'Sensor001.csv';

gpsfid = fopen(gpsfile, 'r');
sensorfid = fopen(sensorfile, 'r');

states = zeros(1,9);
gps_millis = [0];
sensor_millis = 0;

%% First state, start with first GPS position

gps_millis = [str2num(fgetl(gpsfid)) gps_millis];
states = [zeros(1,9); states];
for i = 1:2
    [data, ierr] = nmealineread(fgetl(gpsfid));
    if data.type == 'GGA'
        %Convert lat/long to ENU coordinates and update state
        [x,y,z] = LLA2ENU(data.latitude,data.longitude,data.hWGS);
        states(1,1:3) = [x y z];
    elseif data.type == 'RMC'
        %Convert from knots and set speeds, x is east, y is north 
        
        %THIS MIGHT BE WEIRD IF POSITION IS FAR AWAY FROM REFERENCE POINT OF
        %ENU?
        speed_ms = data.speed * 1.852/3.6;
        states(1,4) = speed_ms * sind(data.truecourse);
        states(1,5) = speed_ms * cosd(data.truecourse);
        
        %interpolate vertical speed as position difference
        states(1,6) = (states(1,3) - states(2,3))/(gps_millis(1) - gps_millis(2))*1000;
    end
    
    %datestr(data.BODCTime, 'dddd mmm dd yyyy HH:MM:SS.FFF')
end

%Skip the first sensor readings until we get past the first GPS fix
while(sensor_millis < gps_millis(1))
    sensor_line = str2num(fgetl(sensorfid));
    sensor_millis = sensor_line(1);
end
%We assume that the Sensor lies flat on the ground and read the first euler
%angle in the direction of the strongest magnetic field, taking that as
%north

%The initial Euler angles will be in reference to the reference point of
%the ENU coordinate system. The y-axis represents north. If the sensor lies
%flat on the ground, the second and third angles should be 0.

%this is not clear nor finished...
states(1,7) = atan2(sensor_line(8),sensor_line(9)) * 180 / pi;
%% Kalman part

188







