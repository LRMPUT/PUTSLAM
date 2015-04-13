function [X,Y] = calculateEllipseMat(pos, mat, steps)
    %# This functions returns points to draw an ellipse
    %#
    %#  @param pos   position
    %#  @param mat   uncertainty matrix
    %#  @param angle Angle of the ellipse (in degrees)
    %#
    x=pos(1);
    y=pos(2);
    
    angle = 0.5*atan2(2*mat(1,2),(mat(1,1)-mat(2,2)));
    tau = sqrt(mat(1,1)^2+mat(2,2)^2-2*mat(1,1)*mat(2,2)+4*mat(1,2));
    k = -2*log(1-99);
    if (mat(1,1)>mat(2,2))
      a = sqrt(k/2*(mat(1,1)+mat(2,2)+tau));
      b = sqrt(k/2*(mat(1,1)+mat(2,2)-tau));
    else
      a = sqrt(k/2*(mat(1,1)+mat(2,2)-tau));
      b = sqrt(k/2*(mat(1,1)+mat(2,2)+tau));
    end
      
    narginchk(3, 6);
    if nargin<6, steps = 36; end

    beta = -angle;
    sinbeta = sin(beta);
    cosbeta = cos(beta);

    alpha = linspace(0, 2*pi, steps)';
    sinalpha = sin(alpha);
    cosalpha = cos(alpha);

    X = x + (a * cosalpha * cosbeta - b * sinalpha * sinbeta);
    Y = y + (a * cosalpha * sinbeta + b * sinalpha * cosbeta);

    if nargout==1, X = [X Y]; end
end