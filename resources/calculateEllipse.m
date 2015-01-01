function [X,Y] = calculateEllipse(x, y, a, b, angle, steps)
    %# This functions returns points to draw an ellipse
    %#
    %#  @param x     X coordinate
    %#  @param y     Y coordinate
    %#  @param a     Semimajor axis
    %#  @param b     Semiminor axis
    %#  @param angle Angle of the ellipse (in degrees)
    %#

    narginchk(5, 6);
    if nargin<6, steps = 36; end

    beta = -angle;
    sinbeta = sin(beta);
    cosbeta = cos(beta);

    alpha = linspace(0, 2*pi, steps)';
    sinalpha = sin(alpha);
    cosalpha = cos(alpha);

    X = x + (a * cosalpha * cosbeta - b * sinalpha * sinbeta);
    Y = y + (a * cosalpha * sinbeta + b * sinalpha * cosbeta);

    C=[b^2*(cos(angle))^2+a^2*(sin(angle))^2, -a^2*cos(angle)*sin(angle)+b^2*sin(angle)*cos(angle);
       -a^2*cos(angle)*sin(angle)+b^2*sin(angle)*cos(angle), a^2*(cos(angle))^2+b^2*(sin(angle))^2]
    if nargout==1, X = [X Y]; end
end