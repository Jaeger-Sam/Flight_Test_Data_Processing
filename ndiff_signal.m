% ndiff_signal.fcn numerically differentiates a signal using a user
% specified method. The default method is to use a 15 point simpson filter
% and a forward differentiation method. Default is to assume the timestep
% is 0.01 s.
%
% zdot = ndiff_signal(z,method,dt,f_max,delta)
%
% INPUTS:
%   z: Nx1 vector of noisy signal
%
% OPTIONAL INPUTS:
%   method: integer input with the following options
%       ==0 forward differentiator
%       ==1 15 point digital low pass filter + forward differentiator
%       ==2 15 point digital low pass filter + central differentiator
%       ==3 freq domain filter + forward differentiator
%       ==4 Total Variation Denoising + forward differentiator
%   dt: timestep
%   f_max: cutoff frequency for low pass filter
%   delta: tuning parameter for TVD
%
% OUTPUTS:
%   zdot: differentiated signal
%   
% Sam Jaeger
% jaege246@umn.edu
% 8/10/2026

function zdot = ndiff_signal(z,varargin)

    narginchk(1,5) % input logic
    if nargin ==2
        method =varargin{1};
        dt = 0.01;
        f_max = [];
        delta = [];
    elseif nargin ==3
        method = varargin{1};
        dt = varargin{2};
        f_max = [];
        delta = [];
    elseif nargin ==4
        method = varargin{1};
        dt = varargin{2};
        f_max = varargin{3};
        delta = [];
    elseif nargin ==5
        method = varargin{1};
        dt = varargin{2};
        f_max = varargin{3};
        delta = varargin{4};
    else
        method = 1;
        dt = 0.01;
        f_max = [];
        delta = [];
    end

    if iscolumn(z) == false % make sure input is column
        z = z';
    end

    N = length(z); 
    if method == 0  % differentiate raw signal
        D0 = sparse(diag(-1*ones(N+2,1),0) + diag(ones(N+1,1),1)); % first order finite diff
        D =  D0(1:(N-1),1:N)*(1/dt); %Remove last two rows + last column, (N-1 x N)

        zdot = D*z;
    elseif method ==1 % 15 pt low digital low pass filter + forward differentiator
        D0 = sparse(diag(-1*ones(N+2,1),0) + diag(ones(N+1,1),1)); % first order finite diff
        D =  D0(1:(N-1),1:N)*(1/dt); %Remove last two rows + last column, (N-1 x N)
        
        y = LP_15smooth(z);
        zdot = D*y;

    elseif method ==2% 15 pt low digital low pass filter + central differentiator
        D0_central = sparse( diag(-1*ones(N+1,1),-1) + diag(ones(N+1,1),1) ); % central difference
        D_c = D0_central(2:(N-1),1:N)*(1/2/dt); % central difference
        
        y = LP_15smooth(z);
        zdot = D_c*y;

    elseif method ==3 % freq domain low pass filter + forward difference
        D0 = sparse(diag(-1*ones(N+2,1),0) + diag(ones(N+1,1),1)); % first order finite diff
        D =  D0(1:(N-1),1:N)*(1/dt); %Remove last two rows + last column, (N-1 x N)

        if isempty(f_max)
            f_max = input('input cutoff frequency in Hz, f_max = ');
        end

        y = Opt_LP(z,f_max,dt,false);
        zdot = D*y;

    elseif method == 4 % TVD + forward differentiator
        if isempty(delta)
            delta = input('input delta tuning parameter for TVD, delta = ');
        end
        D0 = sparse(diag(-1*ones(N+2,1),0) + diag(ones(N+1,1),1)); % first order finite diff
        D =  D0(1:(N-1),1:N)*(1/dt); %Remove last two rows + last column, (N-1 x N)
        zdot = D*((eye(N) + delta*(D'*D))\z);
    end

end