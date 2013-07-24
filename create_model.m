clear all

syms x1 x2 x3 x4 x5 x6 x7 x8 x9 u1 u2 u3 u4 u5 u6 dt real;
x = [x1 x2 x3 x4 x5 x6 x7 x8 x9];
Gyro = [u1 u2 u3];
Acc = [u4 u5 u6];
f = x;
g = 9.80665;

%Next position is last + dt * velocity
f(1:3) = x(1:3) + dt * [x4 x5 x6];

%Calculate the Directional Cosine Matrix between the body frame where the
%acceleration and rotations are measured and the local geodetic frame where
%the position and velocity are defined
DCM1 = [1 0 0;
       0 cos(x7*pi/180) sin(x7*pi/180);
       0 -sin(x7*pi/180) cos(x7*pi/180)];
DCM2 = [cos(x8*pi/180) 0 -sin(x8*pi/180);
        0 1 0;
        sin(x8*pi/180) 0 cos(x8*pi/180)];
DCM3 = [cos(x9*pi/180) sin(x9*pi/180) 0;
        -sin(x9*pi/180) cos(x9*pi/180) 0;
        0 0 1];
DCM=DCM1*DCM2*DCM3;

%The next velocity is the last + acceleration changes.
f(4:6) = x(4:6) + dt * (DCM' * Acc' - [0 0 g]')';


%Rotational changes measured by the gyro
f(7) = x(7) + dt * (Gyro(1) + Gyro(2)*sin(x7*pi/180)*tan(x8*pi/180) + Gyro(3)*cos(x7*pi/180)*tan(x8*pi/180));
f(8) = x(8) + dt * (Gyro(2)*cos(x7*pi/180) - Gyro(3)*sin(x7*pi/180));
f(9) = x(9) + dt * (Gyro(2)*sin(x7*pi/180) + Gyro(3)*cos(x7*pi/180)) * sec(x8*pi/180);

%We need the jacobian to predict the error covariance in the EKF
%formulation
J = jacobian(f,x);

%Save the functions to file
matlabFunction(f,'file','state_transition');
matlabFunction(J,'file','model_jacobian');
%observation_function = matlabFunction(H,'file','observation_function');
