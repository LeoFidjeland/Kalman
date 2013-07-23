clear all;

syms x1 x2 x3 x4 x5 x6 x7 x8 x9 u1 u2 u3 u4 u5 u6 dt real;
v = [x1 x2 x3 x4 x5 x6 x7 x8 x9];
Acc = [u1 u2 u3];
Gyro = [u4 u5 u6];
f = v;
g = 9.80665;

f(1:3) = v(1:3) + dt * [x3 x4 x5];

DCM1 = [1 0 0;
       0 cos(x7) sin(x7);
       0 -sin(x7) cos(x7)];
DCM2 = [cos(x8) 0 -sin(x8);
        0 1 0;
        sin(x8) 0 cos(x8)];
DCM3 = [cos(x9) sin(x9) 0;
        -sin(x9) cos(x9) 0;
        0 0 1];
DCM=DCM1*DCM2*DCM3;

f(4:6) = v(4:6) + dt * (DCM' * Acc' - [0 0 g]')';

f(7) = v(7) + dt * (u4 + u5*sin(x7)*tan(x8) + u6*cos(x7)*tan(x8));
f(8) = v(8) + dt * (u5*cos(x7) - u6*sin(x7));
f(9) = v(9) + dt * (u5*sin(x7) + u6*cos(x7)) * sec(x8);

J = jacobian(f,v)   