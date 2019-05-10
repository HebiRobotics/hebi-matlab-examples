function X = naninterp(X,interpMethod)
    % Interpolate over NaNs
    % See INTERP1 for more info
    if nargin < 2
        interpMethod = 'pchip';
    end
    X(isnan(X)) = interp1(find(~isnan(X)), X(~isnan(X)), find(isnan(X)),interpMethod);
return