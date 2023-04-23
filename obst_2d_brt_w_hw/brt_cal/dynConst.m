function [c,ceq] = dynConst(z,v,xinit,dt)
% non linear dynamic constraints
    c = [];
    N = length(z);
    H = (N-3)/4;
    ceq = zeros(3*H,1);
    % there will be 3*H equality constraints
    for iter = 1:H
        ceq(iter) = z(iter+1)-z(iter)-dt*v*cos(z(2*(H+1)+iter)); % constraint in x
        ceq(H+iter) = z((H+1)+iter+1)-z(H+1+iter)-dt*v*sin(z(2*(H+1)+iter)); % constraint in y
        ceq(2*H+iter) = z(2*(H+1)+iter+1)-z(2*(H+1)+iter)-dt*z(3*(H+1)+iter); % constraint in theta
    end
    % initial state constraints
    ceq = [ceq;z(1)-xinit(1);z(H+2)-xinit(2);z(2*H+3)-xinit(3)];
end