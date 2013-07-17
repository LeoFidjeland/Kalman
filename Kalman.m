clear all
close all

gpsfile = 'NMEA000.csv';
sensorfile = 'Sensor000.csv';

gpsfid = fopen(gpsfile, 'r');
sensorfid = fopen(gpsfile, 'r');






tline = fgetl(gpsfid);
i=0;
while (ischar(tline) && i<100000)
    
    try
        [data, ierr] = nmealineread(tline);
        data.longitude;
    catch exception
        tline = fgetl(gpsfid);
        i = i + 1;
        continue
    end
    
    disp(data.longitude)
    
%   % Measurement update
%   Mn = P*C'/(C*P*C'+R);
%   x = x + Mn*(yv(i)-C*x);  % x[n|n]
%   P = (eye(3)-Mn*C)*P;     % P[n|n]
% 
%   ye(i) = C*x;
%   errcov(i) = C*P*C';
% 
%   % Time update
%   x = A*x + B*u(i);        % x[n+1|n]
%   P = A*P*A' + B*Q*B';     % P[n+1|n]
  
  tline = fgetl(gpsfid);
  i = i + 1;
end