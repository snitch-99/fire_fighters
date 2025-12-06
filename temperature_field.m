function [T, gradT] = temperature_field(x, env)
% TEMPERATURE_FIELD
% Computes temperature T(x) and its gradient gradT(x) given position x
% and environment parameters in env.
%
% Inputs:
%   x   : 2x1 column vector [x; y]
%   env : struct from env_setup()
%
% Outputs:
%   T     : scalar temperature at x
%   gradT : 2x1 gradient vector dT/dx at x

    % Ensure x is a column vector
    if size(x, 2) > 1
        x = x(:);
    end

    % Initialize total temperature and gradient
    T = 0;
    gradT = [0; 0];

    % Loop over all fire hotspots
    for k = 1:env.num_fires
        % Fire parameters
        fk    = env.fire_positions(:, k);   % 2x1 position of fire k
        Ik    = env.fire_intensities(k);    % intensity of fire k
        sigma = env.fire_spreads(k);        % spread (sigma) of fire k

        % Vector from fire to query point
        dx = x - fk;
        r2 = dx' * dx;                      % squared distance ||x - fk||^2

        % Radial bump function ϕ(r) = exp(-r^2 / (2σ^2))
        phi = exp(-r2 / (2 * sigma^2));

        % Temperature contribution from fire k
        Tk = Ik * phi;

        % Gradient of Tk with respect to x:
        % ∇Tk(x) = Tk * ( -1/σ^2 ) * (x - fk)
        gradTk = Tk * ( -dx / (sigma^2) );

        % Accumulate contributions
        T     = T + Tk;
        gradT = gradT + gradTk;
    end
end
