%THETA=[fi psi theta x y z]
close all;
clear all;
symbols;

x = sym ("x");
y = sym ("y");
z = sym ("z");

xa = sym ("xa");
ya = sym ("ya");
za = sym ("za");

xb = sym ("xb");
yb = sym ("yb");
zb = sym ("zb");

fi = sym ("fi");
psi = sym ("psi");
theta = sym ("theta");

k = sym ("k");

%  r11 = Cos(psi)*Cos(theta);
%  r12 = Sin(fi)*Sin(psi)*Cos(theta)-Cos(fi)*Sin(theta);
%  r13 = Cos(fi)*Sin(psi)*Cos(theta)+Sin(fi)*Sin(theta);
%  r21 = Cos(psi)*Sin(theta);
%  r22 = Sin(fi)*Sin(psi)*Sin(theta)+Cos(fi)*Cos(theta);
%  r23 = Cos(fi)*Sin(psi)*Sin(theta)-Sin(fi)*Cos(theta);
%  r31 = 0-Sin(psi);
%  r32 = Sin(fi)*Cos(psi);
%  r33 = Cos(fi)*Cos(psi);
r11 = Cos(psi)*Cos(fi);
r12 = Sin(theta)*Sin(psi)*Cos(fi)-Cos(theta)*Sin(fi);
r13 = Cos(theta)*Sin(psi)*Cos(fi)+Sin(theta)*Sin(fi);
r21 = Cos(psi)*Sin(fi);
r22 = Sin(theta)*Sin(psi)*Sin(fi)+Cos(theta)*Cos(fi);
r23 = Cos(theta)*Sin(psi)*Sin(fi)-Sin(theta)*Cos(fi);
r31 = 0-Sin(psi);
r32 = Sin(theta)*Cos(psi);
r33 = Cos(theta)*Cos(psi);

f = (xa-r11*xb-r12*yb-r13*zb-x)^2+(ya-r21*xb-r22*yb-r23*zb-y)^2+(za-r31*xb-r32*yb-r33*zb-z)^2;

g11 = differentiate (f, fi)
g12 = differentiate (f, psi)
g13 = differentiate (f, theta)
g14 = differentiate (f, x)
g15 = differentiate (f, y)
g16 = differentiate (f, z)

%dg/dTHETA
dg11 = differentiate (g11, fi);
dg12 = differentiate (g12, fi);
dg13 = differentiate (g13, fi);
dg14 = differentiate (g14, fi);
dg15 = differentiate (g15, fi);
dg16 = differentiate (g16, fi);

dg21 = differentiate (g11, psi);
dg22 = differentiate (g12, psi);
dg23 = differentiate (g13, psi);
dg24 = differentiate (g14, psi);
dg25 = differentiate (g15, psi);
dg26 = differentiate (g16, psi);

dg31 = differentiate (g11, theta);
dg32 = differentiate (g12, theta);
dg33 = differentiate (g13, theta);
dg34 = differentiate (g14, theta);
dg35 = differentiate (g15, theta);
dg36 = differentiate (g16, theta);

dg41 = differentiate (g11, x);
dg42 = differentiate (g12, x);
dg43 = differentiate (g13, x);
dg44 = differentiate (g14, x);
dg45 = differentiate (g15, x);
dg46 = differentiate (g16, x);

dg51 = differentiate (g11, y);
dg52 = differentiate (g12, y);
dg53 = differentiate (g13, y);
dg54 = differentiate (g14, y);
dg55 = differentiate (g15, y);
dg56 = differentiate (g16, y);

dg61 = differentiate (g11, z);
dg62 = differentiate (g12, z);
dg63 = differentiate (g13, z);
dg64 = differentiate (g14, z);
dg65 = differentiate (g15, z);
dg66 = differentiate (g16, z);

dgXa11 = differentiate (g11, xa);
dgXa12 = differentiate (g12, xa);
dgXa13 = differentiate (g13, xa);
dgXa14 = differentiate (g14, xa);
dgXa15 = differentiate (g15, xa);
dgXa16 = differentiate (g16, xa);

dgYa11 = differentiate (g11, ya);
dgYa12 = differentiate (g12, ya);
dgYa13 = differentiate (g13, ya);
dgYa14 = differentiate (g14, ya);
dgYa15 = differentiate (g15, ya);
dgYa16 = differentiate (g16, ya);

dgZa11 = differentiate (g11, za);
dgZa12 = differentiate (g12, za);
dgZa13 = differentiate (g13, za);
dgZa14 = differentiate (g14, za);
dgZa15 = differentiate (g15, za);
dgZa16 = differentiate (g16, za);

dgXb11 = differentiate (g11, xb);
dgXb12 = differentiate (g12, xb);
dgXb13 = differentiate (g13, xb);
dgXb14 = differentiate (g14, xb);
dgXb15 = differentiate (g15, xb);
dgXb16 = differentiate (g16, xb);

dgYb11 = differentiate (g11, yb);
dgYb12 = differentiate (g12, yb);
dgYb13 = differentiate (g13, yb);
dgYb14 = differentiate (g14, yb);
dgYb15 = differentiate (g15, yb);
dgYb16 = differentiate (g16, yb);

dgZb11 = differentiate (g11, zb);
dgZb12 = differentiate (g12, zb);
dgZb13 = differentiate (g13, zb);
dgZb14 = differentiate (g14, zb);
dgZb15 = differentiate (g15, zb);
dgZb16 = differentiate (g16, zb);

%Euler angles to qx qy qz reresentation

qx = Sin(fi/2.0)*Cos(psi/2.0)*Cos(theta/2.0)-Cos(fi/2.0)*Sin(psi/2.0)*Sin(theta/2.0);
qy = Cos(fi/2.0)*Sin(psi/2.0)*Cos(theta/2.0)+Sin(fi/2.0)*Cos(psi/2.0)*Sin(theta/2.0);
qz = Cos(fi/2.0)*Cos(psi/2.0)*Sin(theta/2.0)-Sin(fi/2.0)*Sin(psi/2.0)*Cos(theta/2.0);

dqXdFI = differentiate (qx, fi);
dqXdPSI = differentiate (qx, psi);
dqXdTHETA = differentiate (qx, theta);

dqYdFI = differentiate (qy, fi);
dqYdPSI = differentiate (qy, psi);
dqYdTHETA = differentiate (qy, theta);

dqZdFI = differentiate (qz, fi);
dqZdPSI = differentiate (qz, psi);
dqZdTHETA = differentiate (qz, theta);