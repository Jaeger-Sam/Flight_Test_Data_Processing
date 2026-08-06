% regress_aero_coef.fcn performs ordinary-least-squares regression or
% total-least-squares regression for a given set of terms. Covariance is
% estimated based on the residuals of the parameter fit. See Chapter 6 in
% "Flight Vehicle System ID" by Jategaonkar.
%
% [theta_hat, P_est, C_i_est, Jc_opt, param_legend] = regress_aero_coef(C_i, param, u_inf, alpha, beta, delta_e, delta_a, delta_r, delta_f, p, q, r, varargin)
% 
% INPUTS:
%   C_i: Nx1 vector of time history of aerodynamic coefficient
%   param: 19x1 vector of true/false of regressor terms to use 
%       corresponding to {static, linear, mixed quadratic, quadratic}...
%           param=
%           {0,
%           alpha,beta,delta_e,delta_a,delta_r,delta_f,p,q,r,V
%           alpha*beta, alpha^2 *beta, 
%           alpha^2, beta^2, delta_e^2, delta_a^2, delta_r^2, delta_f^2}^T
%   u_inf: Nx1 vector of time history of airspeed in body fixed x (ft/s)
%   alpha: Nx1 vector of time history of angle of attack (deg)
%   beta: Nx1 vector of time history of sideslip (deg)
%   delta_e: Nx1 vector of elevator deflections (deg)
%   delta_a: Nx1 vector of aileron deflections (deg)
%   delta_r: Nx1 vector of rudder deflections (deg)
%   delta_f: Nx1 vector of flap deflections (deg)
%   p: Nx1 vector of roll rates (deg/s)
%   q: Nx1 vector of pitch rates (deg/s)
%   r: Nx1 vector of yaw rates (deg/s)
%
% OPTIONAL INPUTS:
%   param_fix: vector of terms to fix in regressors. If a particular 
%       parameter is to be identified, pass that value in as NaN. If a 
%       numerical value is inputted in param_fix and the associated 
%       parameter is also true in the "param" variable, program will error.
%       Can also pass in [] for no fixed variables.
%   plt_fit: true/false to plot fit time history
%   TLS_tf: true/false to use total-least-squares
%   nondim_unsteday_regress: true/false to nondimensionalize the unsteady
%       regressors
%
% OUTPUTS:
%   theta_hat: estimated parameters
%   P_est: estimated covariance
%   C_i_est: estimated coefficient time history
%   Jc_opt: cost function or residual
%   param_legend: string of parameters that were fitted
%
% Sam Jaeger
% jaege246@umn.edu
% 8/6/2026

function [theta_hat, P_est, C_i_est, Jc_opt, param_legend] = regress_aero_coef(C_i, param, u_inf, alpha, beta, delta_e, delta_a, delta_r, delta_f, p, q, r, varargin)
    
    % variable argument logic----------------------------------------------
    narginchk(12,16)
    if nargin == 13
        param_fix = varargin{1};
        plt_fit = false;
        TLS_tf = false;
        nondim_unsteday_regress = false;
    elseif nargin == 14
        param_fix = varargin{1};
        plt_fit = varargin{2};
        TLS_tf = false;
        nondim_unsteday_regress = false;
    elseif nargin == 15
        param_fix = varargin{1};
        plt_fit = varargin{2};
        TLS_tf = varargin{3};
        nondim_unsteday_regress = false;
    elseif nargin == 16
        param_fix = varargin{1};
        plt_fit = varargin{2};
        TLS_tf = varargin{3};
        nondim_unsteday_regress = varargin{4};
    else
        param_fix = [];
        plt_fit = false;
        TLS_tf = false;
        nondim_unsteday_regress = false;
    end
    if isempty(param_fix) == true
        param_fix = NaN(19,1);
    end
    for ii=1:length(param)
        if param(ii) == 1 && isnan(param_fix(ii)) == false
            error('Cannot both identify and fix parameter! Change param and param_fix inputs.')
        end
    end

    % check if inputs are column vectors-----------------------------------
    if iscolumn(C_i) == false
        C_i = C_i';
    end
    if iscolumn(u_inf) == false
        u_inf = u_inf';
    end
    if iscolumn(alpha) == false
        alpha = alpha';
    end
    if iscolumn(beta) == false
        beta = beta';
    end
    if iscolumn(delta_e) == false
        delta_e = delta_e';
    end
    if iscolumn(delta_a) == false
        delta_a = delta_a';
    end
    if iscolumn(delta_r) == false
        delta_r = delta_r';
    end
    if iscolumn(delta_f) == false
        delta_f = delta_f';
    end
    if iscolumn(p) == false
        p = p';
    end
    if iscolumn(q) == false
        q = q';
    end
    if iscolumn(r) == false
        r = r';
    end

    % convert to radians---------------------------------------------------
    alpha = alpha*pi/180;
    beta = beta*pi/180;
    delta_e = delta_e*pi/180;
    delta_a = delta_a*pi/180;
    delta_r = delta_r*pi/180;
    delta_f = delta_f*pi/180;
    V_inf = u_inf./cos(alpha)./cos(beta);
    N = length(alpha);

    % regessor legend------------------------------------------------------
    var_reg = {'$C_{i_0}$',...
        '$C_{i,\alpha}$',...
        '$C_{i,\beta}$',...
        '$C_{i,{\delta_e}}$',...
        '$C_{i,{\delta_a}}$',...
        '$C_{i,{\delta_r}}$',...
        '$C_{i,{\delta_f}}$',...
        '$C_{i,{p}}$',...
        '$C_{i,{q}}$',...
        '$C_{i,{r}}$',...
        '$C_{i,{V}}$',...
        '$C_{i,{\alpha \beta}}$',...
        '$C_{i,{{\alpha_2} \beta}}$',...
        '$C_{i,\alpha_2}$',...
        '$C_{i,\beta_2}$',...
        '$C_{i,{\delta_e}_2}$',...
        '$C_{i,{\delta_a}_2}$',...
        '$C_{i,{\delta_r}_2}$',...
        '$C_{i,{\delta_f}_2}$'};

    % formulate regressors-------------------------------------------------
    H_0 = ones(N,1);
    H_a = alpha;
    H_b = beta;
    H_de = delta_e;
    H_da = delta_a;
    H_dr = delta_r;
    H_df = delta_f;
    if nondim_unsteday_regress == true
        c_b_w = input('input value for mean aero chord in ft, c_b_w = ');
        b_w = input('input value for wingspan  in ft, b_w = ');
        H_p = r*b_w./V_inf;
        H_q = q*c_b_w./V_inf;
        H_r = r*b_w./V_inf;
    else
        H_p = p;
        H_q = q;
        H_r = r;
    end
    H_V = V_inf;
    H_ab = alpha.*beta;
    H_a2b = (alpha.^2).*beta;
    H_a2 = (alpha.^2);
    H_b2 = (beta.^2);
    H_de2 = delta_e.^2;
    H_da2 = delta_a.^2;
    H_dr2 = delta_r.^2;
    H_df2 = delta_f.^2;
    H_full = [H_0,H_a,H_b,H_de,H_da,H_dr,H_df,H_p,H_q,H_r,H_V,H_ab,H_a2b,H_a2,H_b2,H_de2,H_da2,H_dr2,H_df2];

    % remove columns based on parameters to be fit
    H = [];
    count = 1;
    for ii=1:length(param)
        if param(ii) == 1
            H(:,count) = H_full(:,ii);
            count = count + 1;
        end
    end

    % RHS of linear system of equations
    y = C_i;

    % removed fixed parameters from RHS of linear system of equation
    for ii=1:length(param_fix)
        if isnan(param_fix(ii)) == false
            y = y - param_fix(ii)*H_full(:,ii);
        end
    end

    % perform regression---------------------------------------------------
    if TLS_tf == true % total least squares
        disp('===================================')
        disp('======= TOTAL LEAST-SQUARES =======')
        % Eqn 6.52 in Flight Vehicle System ID
        sig_nz = svds([H y],1,'smallestnz'); % smallest nonzero sigular value
        theta_hat = inv(H'*H - (sig_nz^2)*eye(length(H(1,:))))*H'*y;
        
        % % code copied from the internet to check...
        % [~,n]   = size(H);
        % Z       = [H y];               % Z is X augmented with Y.
        % [~,~,V] = svd(Z, 0);           % find the SVD of Z.
        % VXY     = V(1:n, 1+n:end);     % Take the block of V consisting of the first n rows and the n+1 to last column
        % VYY     = V(1+n:end, 1+n:end); % Take the bottom-right block of V.
        % theta_hat       = -VXY / VYY; % gives the same answer as 6.52

        C_i_est = H*theta_hat;
        Jc_opt = norm((C_i_est - y),2); % this isn't right for TLS
        count = 1;
        param_legend = [];
        for ii=1:length(param)
            if param(ii) == 1
                param_legend(ii) = string(var_reg(ii));
                disp(append(string(var_reg(ii)),' = ',num2str(theta_hat(count)) ) )
                count = count+1;
            end
        end
        disp(append('---------- Jc = ',num2str(Jc_opt),'-----------'))
        % uncertainty (follow Eqns. 6.21, 6.22 in Flight Vehicle Sys ID)
        %   this isn't right for TLS...
        sig_hat_2 = (Jc_opt^2)./(N - sum(param)); 
        P_est = (sig_hat_2/N)*inv(H'*H*(1/N));

    else % ordinary least squares
        disp('===================================')
        disp('===== ORDINARY LEAST-SQUARES ======')
        theta_hat = (H'*H)\H'*y;
        C_i_est = H*theta_hat;
        Jc_opt = norm((C_i_est - y),2);
        count = 1;
        param_legend = [];
        for ii=1:length(param)
            if param(ii) == 1
                param_legend(ii) = string(var_reg(ii));
                disp(append(string(var_reg(ii)),' = ',num2str(theta_hat(count)) ) )
                count = count+1;
            end
        end
        disp(append('---------- Jc = ',num2str(Jc_opt),'-----------'))
        
        % uncertainty (follow Eqns. 6.21, 6.22 in Flight Vehicle Sys ID)
        sig_hat_2 = (Jc_opt^2)./(N - sum(param));
        P_est = (sig_hat_2/N)*inv(H'*H*(1/N));
    end

    % plot fit-------------------------------------------------------------
    if plt_fit == true
        figure(950)
        plot(y,'*'); hold on
        plot(C_i_est,'.')
        xlabel('timestep','FontSize',15,'Interpreter','latex')
        ylabel('$y$','FontSize',15,'Interpreter','latex');
        legend('Measured $C_i$','Fitted $C_i$','Interpreter','latex')
        grid on;
        hold off;
    end
end