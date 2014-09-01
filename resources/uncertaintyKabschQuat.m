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

qx = sym ("qx");
qy = sym ("qy");
qz = sym ("qz");
qw = sym ("qw");

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
r11 = 1 - (2*(qy^2)) - (2*(qz^2));
r12 = (2*qx*qy)-(2*qz*qw);
r13 = (2*qx*qz)+(2*qy*qw);
r21 = (2*qx*qy)+(2*qz*qw);
r22 = 1 - (2*(qx^2)) - (2*(qz^2));
r23 = (2*qy*qz)-(2*qx*qw);
r31 = (2*qx*qz)-(2*qy*qw);
r32 = (2*qy*qz)+(2*qx*qw);
r33 = 1 - (2*(qx^2)) - (2*(qy^2));

f = (xa-r11*xb-r12*yb-r13*zb-x)^2+(ya-r21*xb-r22*yb-r23*zb-y)^2+(za-r31*xb-r32*yb-r33*zb-z)^2;

g11 = differentiate (f, x);
g12 = differentiate (f, y);
g13 = differentiate (f, z);
g14 = differentiate (f, qx);
g15 = differentiate (f, qy);
g16 = differentiate (f, qz);

%dg/dTHETA
dg11 = differentiate (g11, x);
dg12 = differentiate (g12, x);
dg13 = differentiate (g13, x);
dg14 = differentiate (g14, x);
dg15 = differentiate (g15, x);
dg16 = differentiate (g16, x);

dg21 = differentiate (g11, y);
dg22 = differentiate (g12, y);
dg23 = differentiate (g13, y);
dg24 = differentiate (g14, y);
dg25 = differentiate (g15, y);
dg26 = differentiate (g16, y);

dg31 = differentiate (g11, z);
dg32 = differentiate (g12, z);
dg33 = differentiate (g13, z);
dg34 = differentiate (g14, z);
dg35 = differentiate (g15, z);
dg36 = differentiate (g16, z);

dg41 = differentiate (g11, qx);
dg42 = differentiate (g12, qx);
dg43 = differentiate (g13, qx);
dg44 = differentiate (g14, qx);
dg45 = differentiate (g15, qx);
dg46 = differentiate (g16, qx);

dg51 = differentiate (g11, qy);
dg52 = differentiate (g12, qy);
dg53 = differentiate (g13, qy);
dg54 = differentiate (g14, qy);
dg55 = differentiate (g15, qy);
dg56 = differentiate (g16, qy);

dg61 = differentiate (g11, qz);
dg62 = differentiate (g12, qz);
dg63 = differentiate (g13, qz);
dg64 = differentiate (g14, qz);
dg65 = differentiate (g15, qz);
dg66 = differentiate (g16, qz);

dgXa11 = differentiate (g11, xa)
dgXa12 = differentiate (g12, xa)
dgXa13 = differentiate (g13, xa)
dgXa14 = differentiate (g14, xa)
dgXa15 = differentiate (g15, xa)
dgXa16 = differentiate (g16, xa)

dgYa11 = differentiate (g11, ya)
dgYa12 = differentiate (g12, ya)
dgYa13 = differentiate (g13, ya)
dgYa14 = differentiate (g14, ya)
dgYa15 = differentiate (g15, ya)
dgYa16 = differentiate (g16, ya)

dgZa11 = differentiate (g11, za)
dgZa12 = differentiate (g12, za)
dgZa13 = differentiate (g13, za)
dgZa14 = differentiate (g14, za)
dgZa15 = differentiate (g15, za)
dgZa16 = differentiate (g16, za)

dgXb11 = differentiate (g11, xb)
dgXb12 = differentiate (g12, xb)
dgXb13 = differentiate (g13, xb)
dgXb14 = differentiate (g14, xb)
dgXb15 = differentiate (g15, xb)
dgXb16 = differentiate (g16, xb)

dgYb11 = differentiate (g11, yb)
dgYb12 = differentiate (g12, yb)
dgYb13 = differentiate (g13, yb)
dgYb14 = differentiate (g14, yb)
dgYb15 = differentiate (g15, yb)
dgYb16 = differentiate (g16, yb)

dgZb11 = differentiate (g11, zb)
dgZb12 = differentiate (g12, zb)
dgZb13 = differentiate (g13, zb)
dgZb14 = differentiate (g14, zb)
dgZb15 = differentiate (g15, zb)
dgZb16 = differentiate (g16, zb)
