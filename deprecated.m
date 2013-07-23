   %dt = interval;
    %pred = zeros(1,9);
    %new position is last + velocity * dt
    %pred(1:3) = states(1,1:3) + states(1,4:6) * dt;
    
    
%     pred(4) = states(1,4) + dt * ( ...
%         Acc(1) * cosd(e3)*cosd(e2) + ...
%         Acc(2) * (-sind(e3)*cosd(e1) + cosd(e3)*sind(e2)*sind(e1)) + ...
%         Acc(3) * (sind(e3)*sind(e1) + cosd(e3)*sind(e2)*cosd(e1)) );
%     pred(5) = states(1,5) + dt * ( ...
%         Acc(1) * sind(e3)*cosd(e2) + ...
%         Acc(2) * (cosd(e3)*cosd(e1) + sind(e3)*sind(e2)*sind(e1)) + ...
%         Acc(3) * (-cosd(e3)*sind(e1) + sind(e3)*sind(e2)*cosd(e1)) );
%     pred(6) = states(1,6) + dt * ( ...
%         Acc(1) * (-sind(e2)) + ...
%         Acc(2) * (cosd(e2)*sind(e1)) + ...
%         Acc(3) * (cosd(e2)*cosd(e1)) ...
%         - g);
    
    %Euler angles and their DCM
%     e1 = states(1,7);
%     e2 = states(1,8);
%     e3 = states(1,9);
%     DCM1 = [1 0 0;
%         0 cosd(e1) sind(e1);
%         0 -sind(e1) cosd(e1)];
%     DCM2 = [cosd(e2) 0 -sind(e2);
%         0 1 0;
%         sind(e2) 0 cosd(e2)];
%     DCM3 = [cosd(e3) sind(e3) 0;
%         -sind(e3) cosd(e3) 0;
%         0 0 1];
%     %transforms body frame to local geodetic frame
%     DCM = DCM1*DCM2*DCM3;
%     
%     %New velocity is last + dt * acc in geodetic frame
%     pred(4:6) = states(1,4:6) + dt * (DCM' * Acc' - [0 0 g]')';
%     
%     %Roll, pitch and yaw from gyro readings
%     r = Gyro(1);
%     p = Gyro(2);
%     y = Gyro(3);
%     
%     pred(7) = states(1,7) + dt * (r + p*sind(e1)*tand(e2) + y*cosd(e1)*tand(e2));
%     pred(8) = states(1,8) + dt * (p*cosd(e1) - y*sind(e1));
%     pred(9) = states(1,9) + dt * (p*sind(e1) + y*cosd(e1)) * secd(e2);