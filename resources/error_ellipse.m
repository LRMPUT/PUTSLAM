function h=error_ellipse(C, M)
%  C = [4.6845, -1.8587, 1.6523; -1.8587, 1.3192, -0.7436; 1.6523, -0.7436, 1.2799];
%  M = [-9.7275; 4.5526; -5.6775];

[U,L] = eig(C);

% For N standard deviations spread of data, the radii of the eliipsoid will
% be given by N*SQRT(eigenvalues).

N = 1; % choose your own N
radii = N*sqrt(diag(L));

% generate data for "unrotated" ellipsoid
[xc,yc,zc] = ellipsoid(0,0,0,radii(1),radii(2),radii(3));

% rotate data with orientation matrix U and center M
a = kron(U(:,1),xc); b = kron(U(:,2),yc); c = kron(U(:,3),zc);
data = a+b+c; n = size(data,2);
x = data(1:n,:)+M(1); y = data(n+1:2*n,:)+M(2); z = data(2*n+1:end,:)+M(3);

% now plot the rotated ellipse
sc = mesh(x,y,z,'EdgeColor','red');
set(sc,'facecolor','none');