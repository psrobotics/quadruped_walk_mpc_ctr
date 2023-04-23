function uOpt = optCtrl(obj, ~, ~, deriv, uMode)
% uOpt = optCtrl(obj, t, y, deriv, uMode)

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% TODO 
% Compute the optimal controller 
uOpt = obj.wRange(2)*(deriv{3} >= 0) - obj.wRange(2)*(deriv{3} < 0);
