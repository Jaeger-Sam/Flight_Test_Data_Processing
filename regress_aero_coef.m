% regress_aero_coef.fcn performs ordinary-least-squares regression or
% total-least-squares regression for a given set of terms. Covariance is
% estimated based on the residuals of the parameter fit. See Chapter 6 in
% "Flight Vehicle System ID" by Jategaonkar and also Chapter 5 of "Aircraft
% System Identification" by Morelli. All parameter estimates are in rad.
%
% [theta_hat, P_est, C_i_est, Jc_opt, param_legend,PSE,R2,C_num,VIF] = regress_aero_coef(C_i, param, u_inf, alpha, beta, delta_e, delta_a, delta_r, delta_f, p, q, r, varargin)
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
%   c_b_w: mean aerodynamic chord for nondimensional unsteady regressors
%   b_w: wingspan for nondimensional unsteady regressors
%   filt_regressors: true / false to filter regressors via 15 point simpson
%       filter.
%   stand_regressors: true / false to transform regression problem into a
%       nondimensional form.
%   i_maneuver: index of different maneuvers to plot
%   maneuver_label: string of maneuver labels to add to plot
%
% OUTPUTS:
%   theta_hat: estimated parameters
%   P_est: estimated covariance matrix
%   C_i_est: estimated coefficient time history
%   Jc_opt: cost function or residual 
%   param_legend: string of parameters that were fitted
%
% OPTIONAL OUTPUTS:
%   PSE: Predicted Square Error (Eqn 5.115 in Morelli)
%         small number indicates a good fit without overfitting parameters.
%   R2: Coefficient of determination (Eqn. 5.31 in Morelli) 
%         value close to 1 indicates a good fit.
%   C_num: Condition number of regressor matrix (Eqn. 5.174 in Morelli)
%          value greater than 100 indicates collinearity and inverse of 
%          H'*H may not exist.
%   VIF: Vector of variance inflation factors (Eqn 5.168 in Morelli) 
%       if value >10 points to collinearity in regressor.
%   U: Theil's Inequality Coefficient (Eqn 11.5 in Jategaonkar)
%       value close to 0 indicates a good fit.
%
% Sam Jaeger
% jaege246@umn.edu
% 8/6/2026

function [theta_hat, P_est, C_i_est, Jc_opt, param_legend,varargout] = regress_aero_coef(C_i, param, u_inf, alpha, beta, delta_e, delta_a, delta_r, delta_f, p, q, r, varargin)
    
    % variable argument logic----------------------------------------------
    narginchk(12,22)
    if nargin == 13
        param_fix = varargin{1};
        plt_fit = false;
        TLS_tf = false;
        nondim_unsteday_regress = false;
        c_b_w = [];
        b_w = [];
        filt_regressors = false;
        stand_regressors = false;
        i_maneuver = [];
        maneuver_label = [];
    elseif nargin == 14
        param_fix = varargin{1};
        plt_fit = varargin{2};
        TLS_tf = false;
        nondim_unsteday_regress = false;
        c_b_w = [];
        b_w = [];
        filt_regressors = false;
        stand_regressors = false;
        i_maneuver = [];
        maneuver_label = [];
    elseif nargin == 15
        param_fix = varargin{1};
        plt_fit = varargin{2};
        TLS_tf = varargin{3};
        nondim_unsteday_regress = false;
        c_b_w = [];
        b_w = [];
        filt_regressors = false;
        stand_regressors = false;
        i_maneuver = [];
        maneuver_label = [];
    elseif nargin == 16
        param_fix = varargin{1};
        plt_fit = varargin{2};
        TLS_tf = varargin{3};
        nondim_unsteday_regress = varargin{4};
        c_b_w = [];
        b_w = [];
        filt_regressors = false;
        stand_regressors = false;
        i_maneuver = [];
        maneuver_label = [];
    elseif nargin == 17
        param_fix = varargin{1};
        plt_fit = varargin{2};
        TLS_tf = varargin{3};
        nondim_unsteday_regress = varargin{4};
        c_b_w = varargin{5};
        b_w = [];
        filt_regressors = false;
        stand_regressors = false;
        i_maneuver = [];
        maneuver_label = [];
    elseif nargin == 18
        param_fix = varargin{1};
        plt_fit = varargin{2};
        TLS_tf = varargin{3};
        nondim_unsteday_regress = varargin{4};
        c_b_w = varargin{5};
        b_w = varargin{6};
        filt_regressors = false;
        stand_regressors = false;
        i_maneuver = [];
        maneuver_label = [];
    elseif nargin == 19
        param_fix = varargin{1};
        plt_fit = varargin{2};
        TLS_tf = varargin{3};
        nondim_unsteday_regress = varargin{4};
        c_b_w = varargin{5};
        b_w = varargin{6};
        filt_regressors = varargin{7};
        stand_regressors = false;
        i_maneuver = [];
        maneuver_label = [];
    elseif nargin == 20
        param_fix = varargin{1};
        plt_fit = varargin{2};
        TLS_tf = varargin{3};
        nondim_unsteday_regress = varargin{4};
        c_b_w = varargin{5};
        b_w = varargin{6};
        filt_regressors = varargin{7};
        stand_regressors = varargin{8};
        i_maneuver = [];
        maneuver_label = [];
    elseif nargin == 21
        param_fix = varargin{1};
        plt_fit = varargin{2};
        TLS_tf = varargin{3};
        nondim_unsteday_regress = varargin{4};
        c_b_w = varargin{5};
        b_w = varargin{6};
        filt_regressors = varargin{7};
        stand_regressors = varargin{8};
        i_maneuver = varargin{9};
        maneuver_label = [];
    elseif nargin == 22
        param_fix = varargin{1};
        plt_fit = varargin{2};
        TLS_tf = varargin{3};
        nondim_unsteday_regress = varargin{4};
        c_b_w = varargin{5};
        b_w = varargin{6};
        filt_regressors = varargin{7};
        stand_regressors = varargin{8};
        i_maneuver = varargin{9};
        maneuver_label = varargin{10};
    else
        param_fix = [];
        plt_fit = false;
        TLS_tf = false;
        nondim_unsteday_regress = false;
        c_b_w = [];
        b_w = [];
        filt_regressors = false;
        stand_regressors = false;
        i_maneuver = [];
        maneuver_label = [];
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
    p = p*pi/180;
    q = q*pi/180;
    r = r*pi/180;
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

    % for fprintf
    var_reg_0 ={'C_{i_0} - - - - - - - ',...
                'C_{i,alpha} - - - - - ',...
                'C_{i,beta}  - - - - - ',...
                'C_{i,{delta_e}} - - - ',...
                'C_{i,{delta_a}} - - - ',...
                'C_{i,{delta_r}} - - - ',...
                'C_{i,{delta_f}} - - - ',...
                'C_{i,{p}} - - - - - - ',...
                'C_{i,{q}} - - - - - - ',...
                'C_{i,{r}} - - - - - - ',...
                'C_{i,{V}} - - - - - - ',...
                'C_{i,{alpha beta}}  - ',...
                'C_{i,{{alpha_2} beta}}',...
                'C_{i,alpha_2} - - - - ',...
                'C_{i,beta_2}  - - - - ',...
                'C_{i,{delta_e}_2} - - ',...
                'C_{i,{delta_a}_2} - - ',...
                'C_{i,{delta_r}_2} - - ',...
                'C_{i,{delta_f}_2} - - '};

    % formulate regressors-------------------------------------------------
    H_0 = ones(N,1);
    H_a = alpha;
    H_b = beta;
    H_de = delta_e;
    H_da = delta_a;
    H_dr = delta_r;
    H_df = delta_f;
    if nondim_unsteday_regress == true
        if isempty(c_b_w)
            c_b_w = input('input value for mean aero chord in ft, c_b_w = ');
        end
        if isempty(b_w)
            b_w = input('input value for wingspan  in ft, b_w = ');
        end
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
            if filt_regressors == true
                H(:,count) = LP_15smooth(H_full(:,ii));
            else
                H(:,count) = H_full(:,ii);
            end
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
        disp('========================================================')
        disp('================= TOTAL LEAST-SQUARES ==================')
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
        eps = (C_i_est - y); % residual
        Jc_opt = norm(eps,2); % this isn't right for TLS
        
        % uncertainty (follow Eqns. 6.21, 6.22 in Flight Vehicle Sys ID)
        %   this isn't right for TLS...
        sig_hat_2 = (Jc_opt^2)./(N - sum(param)); 
        P_est = (sig_hat_2/N)*inv(H'*H*(1/N));

        % Predicted Square Error (PSE)
        p_num = length(theta_hat); %number of parameters
        sig_2_max = std(y).^2; % eqn 5.117 Morelli (conservative estimate)
        PSE = eps'*eps/N + sig_2_max*p_num/N; % eqn 5.115 Morelli
        
        % R^2 coefficient (Coefficient of determination)
        R2 = (theta_hat'*H'*y - (N*mean(y).^2))./(y'*y - N*(mean(y).^2));

        % Condition number
        C_num = cond(H);
        if C_num > 100
            warning('Large condition number! Collinearity in regressors is likely present.')
        end

        % VIF
        VIF = zeros(length(H(1,:)),1);
        for jj=1:length(VIF)
            VIF(jj) = 1/(1 - (theta_hat(jj)'*H(:,jj)'*y - (N*mean(y).^2))./(y'*y - N*(mean(y).^2)));
            if VIF(jj) > 10 
                warning(append('Large VIF in Regressor # = ',num2str(jj),'. Likely collinear!'))
            end
        end

        % Theil's Inequality Coefficient
        %   Eqn 11.5 in Flight Vehicle System ID
        U = sqrt(mean(eps.^2))./(sqrt(mean(y.^2)) + sqrt(mean(C_i_est.^2))); 


        % print out estimates
        count = 1;
        param_legend = [];
        disp('PARAMETER ------------ |  ESTIMATE  |  3 SIGMA  |  VIF  ')
        for ii=1:length(param)
            if param(ii) == 1
                param_legend(ii) = string(var_reg(ii));
                %disp(append(string(var_reg(ii)),' = ',num2str(theta_hat(count)) ) )
                
                formatSpec = append(string(string(var_reg_0(ii))),' = %+8.7f | %8.7f | %6.4f \n');
                sig_3 = 3*sqrt((P_est(count,count)));
                fprintf(formatSpec,theta_hat(count),sig_3,VIF(count) )

                count = count+1;
            end
        end


        disp(append('------------- Condition Number = ',num2str(C_num),  '----------------'))
        disp(append('-------------------- R^2 = ',num2str(R2),  '----------------------'))
        disp(append('------------------- PSE = ',num2str(PSE),  '--------------------'))
        disp(append('--------------------- Jc = ',num2str(Jc_opt),'----------------------'))
        disp('========================================================')

    else % ordinary least squares
        disp('========================================================')
        disp('================ ORDINARY LEAST-SQUARES ================')
        
        if stand_regressors == true
            if param(1) == 1
                error('cannot include static term in normalization')
            end
            % normalize H matrix before solving
            [H_N,C,S] = normalize(H);
            H_N = H_N + C./S; % remove centering term in scaled H matrix
            theta_hat = ((H_N'*H_N)\H_N'*y)./S';
            C_i_est = H*theta_hat;
        else
            theta_hat = (H'*H)\H'*y; % estimate of parameters
            C_i_est = H*theta_hat; % estimate of coefficient
        end
        
        eps = (C_i_est - y);
        Jc_opt = norm(eps,2); % 2 norm of residual

        % Predicted Square Error (PSE)
        p_num = length(theta_hat); %number of parameters
        sig_2_max = std(y).^2; % eqn 5.117 Morelli (conservative estimate)
        PSE = eps'*eps/N + sig_2_max*p_num/N; % eqn 5.115 Morelli

        % R^2 coefficient (Coefficient of determination)
        R2 = (theta_hat'*H'*y - (N*mean(y).^2))./(y'*y - N*(mean(y).^2));

        % condition number
        sig_val = svd(H);
        C_num = max(sig_val)/min(sig_val(sig_val>0));
        %C_num = cond(H); Equivalent to the singular value formula above
        if C_num > 100
            warning('Large condition number! Collinearity in regressors is likely present.')
        end

        % VIF 
        VIF = zeros(length(H(1,:)),1);
        for jj=1:length(VIF)
            VIF(jj) = 1/(1 - (theta_hat(jj)'*H(:,jj)'*y - (N*mean(y).^2))./(y'*y - N*(mean(y).^2)));
            if VIF(jj) > 10 
                warning(append('Large VIF in Regressor # = ',num2str(jj),'. Likely collinear!'))
            end
        end

        % Theil's Inequality Coefficient
        %   Eqn 11.5 in Flight Vehicle System ID
        U = sqrt(mean(eps.^2))./(sqrt(mean(y.^2)) + sqrt(mean(C_i_est.^2))); 

        % uncertainty (follow Eqns. 6.21, 6.22 in Flight Vehicle Sys ID)
        sig_hat_2 = (Jc_opt^2)./(N - sum(param));
        P_est = (sig_hat_2/N)*inv(H'*H*(1/N));

        % print out variable estimates
        count = 1;
        param_legend = [];
        disp('PARAMETER ------------ |  ESTIMATE  |  3 SIGMA  |  VIF  ')
        for ii=1:length(param)
            if param(ii) == 1
                param_legend(ii) = string(var_reg(ii));
                %disp(append(string(var_reg(ii)),' = ',num2str(theta_hat(count)) ) )
                
                
                formatSpec = append(string(string(var_reg_0(ii))),' = %+8.7f | %8.7f | %6.4f \n');
                sig_3 = 3*sqrt((P_est(count,count)));
                fprintf(formatSpec,theta_hat(count),sig_3,VIF(count) )
                count = count+1;
            end
        end

        

        disp(append('------------- Condition Number = ',num2str(C_num),  '----------------'))
        disp(append('-------------------- R^2 = ',num2str(R2),  '----------------------'))
        disp(append('------------------- PSE = ',num2str(PSE),  '--------------------'))
        disp(append('----------- Theils Inequality Coef = ',num2str(U),  '------------'))
        disp(append('--------------------- Jc = ',num2str(Jc_opt),'---------------------'))
        disp('========================================================')
    end

    % optional outputs
    varargout{1} = PSE;
    varargout{2} = R2;
    varargout{3} = C_num; 
    varargout{4} = VIF;
    varargout{5} = U;


    % plot fit-------------------------------------------------------------
    if plt_fit == true
        figure(950)
        plot(y,'*'); hold on
        plot(C_i_est,'.')
        xlabel('timestep','FontSize',20,'Interpreter','latex')
        ylabel('$y$','FontSize',20,'Interpreter','latex');
        legend('Measured $C_i$','Fitted $C_i$','Interpreter','latex','FontSize',15)
        grid on;
        if isempty(i_maneuver) == false && length(i_maneuver) > 1
            xline(i_maneuver,'-',maneuver_label,'Interpreter','none','HandleVisibility','off','LabelHorizontalAlignment','left')
        end
        hold off;
    end
end