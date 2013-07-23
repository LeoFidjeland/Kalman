clear all
close all

gpsfile = 'NMEA001.csv';
sensorfile = 'Sensor001.csv';

gpsfid = fopen(gpsfile, 'r');
sensorfid = fopen(sensorfile, 'r');

g = 9.80665; %gravity

states = zeros(1,9);
gps_millis = [0];
sensor_millis = [0];
estimated = zeros(1,3);

euler = zeros(1,3);
first_run = 1;
weight=5; %euler dynamic tilt, higher weight trusts gyro more


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

%% Calculate initial Euler angles by tilt
%Keep on calculating the Euler angles and tilt-compensated heading until we
%get the first GPS fix, then we use these as the starting value of the
%Kalman filter

%this is based on the Ultimate IMU implementation which in turn is based on
%this guide http://www.starlino.com/imu_guide.html

sensor_line = str2num(fgetl(sensorfid));
sensor_millis = [sensor_line(1) sensor_millis];
while(sensor_millis(1) < gps_millis(1))
    interval = sensor_millis(1) - sensor_millis(2);
    
    %Populate the vectors
    Gyro = [sensor_line(2) sensor_line(3) sensor_line(4)];
    Acc = [sensor_line(5) sensor_line(6) sensor_line(7)];
    Mag = [sensor_line(8) sensor_line(9) sensor_line(10)];
    
    Acc = Acc / norm(Acc);
    
    if(first_run)
        %The measurements of the first run get screwed up since we don't have a previous angle to 
        %derive the current angle from. So, we skip the calculation on the first 
        %time.
        GyroVec = Acc;
        first_run = 0;
    else
        %If the previous estimated values is too small, don't calc. a new one as the error will be large.
		if(estimated(3) < 0.1)
			GyroVec = estimated;
        else %Else, find the 'gyro angle' and calculate the weighted average to find attitude of device.
            %find the Angle difference between the last reading and this one.
            
            %We use the euler vector just as a placeholder here, not really
            %kosher
            euler = Gyro * interval / 1000;
        
            %Find the current angle based on the previously measured angle
            euler(1) = euler(1) + atan2(estimated(1),estimated(3))*180/pi;
            euler(2) = euler(2) + atan2(estimated(2),estimated(3))*180/pi;
        end
        
        %Use the angles to find the gravity vector
        GyroVec(1) = sind(euler(1)) / sqrt(1 + (cosd(euler(1)))^2*(tand(euler(2)))^2);
        GyroVec(2) = sind(euler(2)) / sqrt(1 + (cosd(euler(2)))^2*(tand(euler(1)))^2);
        GyroVec(3) = sqrt(1 - (GyroVec(1))^2 - (GyroVec(2))^2);
        
        if(Acc(3) < 0)
            GyroVec(3) = - GyroVec(3);
        end
    end
    
    %Now we have the gravity force vector from both accelerometer and gyro.
    %Combine them using weighted average to find the tilt
    estimated = Acc + GyroVec*weight / (1+weight);
    estimated = estimated / norm(estimated);
    
    euler(1) = atan2(estimated(1),estimated(3))*180/pi;
    euler(2) = atan2(estimated(2),estimated(3))*180/pi;
    
    %Now that we have the euler angles, calculate the tilt-compensated heading.
    xheading = Mag(1)*cosd(euler(2)) + Mag(2)*sind(euler(1))*sind(euler(2)) - Mag(3)*cosd(euler(1))*sind(euler(2));
    yheading = Mag(2)*cosd(euler(1)) + Mag(3)*sind(euler(1));
    
    %find the heading
    if xheading < 0
        euler(3) = 180 - atan2(yheading,xheading)*180/pi;
    elseif xheading > 0 && yheading < 0
        euler(3) = -atan2(yheading,xheading)*180/pi;
    elseif xheading > 0 && yheading > 0
        euler(3) = 360 - atan2(yheading,xheading)*180/pi;
    elseif xheading == 0 && yheading < 0
        euler(3) = 90;
    elseif xheading == 0 && yheading > 0
        euler(3) = 270;
    end
    
    %Read next line and start over!
    sensor_line = str2num(fgetl(sensorfid));
    sensor_millis = [sensor_line(1) sensor_millis];
end

%Use the last angles of the tilt estimator for initial kalman angles
states(1,7:9) = [euler(1) euler(2) euler(3)];
last_time = gps_millis(1);
%% Kalman part

syms x1 x2 x3 x4 x5 x6 x7 x8 x9 u1 u2 u3 u4 u5 u6 dt real;
x = [x1 x2 x3 x4 x5 x6 x7 x8 x9];
u = [u1 u2 u3 u4 u5 u6];
[J,f] = flight_model();

if(sensor_millis(1) > gps_millis(1)) %predict
    
    %Read out the new values
    interval = (sensor_millis(1) - last_time) / 1000;
    Gyro = [sensor_line(2) sensor_line(3) sensor_line(4)];
    Acc = g * [sensor_line(5) sensor_line(6) sensor_line(7)];
    
    %predict the next state
    pred = subs(f,[x u dt], [states(1,:) Gyro Acc interval]);
       
    %Project the error covariance ahead
    A = subs(J,[x u dt], [states(1,:) Gyro Acc interval]);
    
    
    states = [pred; states];
    last_time = sensor_millis(1);
    
    %Go to next line
    sensor_line = str2num(fgetl(sensorfid));
    sensor_millis = [sensor_line(1) sensor_millis];
    
else %correct
    
end




