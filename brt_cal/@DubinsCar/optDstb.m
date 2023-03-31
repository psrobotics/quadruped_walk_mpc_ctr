function dOpt = optDstb(obj, ~, ~, deriv, dMode)
% dOpt = optCtrl(obj, t, y, deriv, dMode)
%     Dynamics of the DubinsCar
%         \dot{x}_1 = v * cos(x_3) + d_1
%         \dot{x}_2 = v * sin(x_3) + d_2
%         \dot{x}_3 = u

%% Input processing
if nargin < 5
  dMode = 'max';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

dOpt = cell(obj.nd, 1);

%% TODO 
% Compute the optimal disturbance 
opt_theta_map = pi - atan2(deriv{2},deriv{1}); % smallest case
dOpt{1} = 1 * obj.dMax * cos(opt_theta_map); %x
dOpt{2} = -1 * obj.dMax * sin(opt_theta_map); %y

end