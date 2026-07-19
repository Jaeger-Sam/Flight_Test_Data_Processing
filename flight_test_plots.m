% flight_test_plots generates time history and position plots from the
% data_int data structure from process_flight_test_data. 
% This function operates similarly to
% SIMULATION_PLOTS.fcn in Sam's Flight Simulator. 
%
% INPUTS:
%   data_int: data structure from process_flight_test_data.fcn
%
% OUTPUTS: generated the following figures
%   figure(101): North-East-Down position 3d plot (Hampel filter applied)
%   figure(1001): GPS position zoomed to the ACRC field
%   figure(10):  uvw from FMUR air data
%   figure(11):  pqr from n580
%   figure(12):  attitude (roll/pitch/yaw) from n580 (x axis linked)
%   figure(13):  acceleration from n580
%   figure(14): aero angle (alpha, beta, beta_f)
%   figure(15): control time history (rpm, delta_e, delta_a, delta_r)
%   figure(20): alpha / q / delta_e (x axis linked)
%   figure(21): beta / p / delta_a (x axis linked)
%   figure(22): beta / r / delta_r (x axis linked)
%   figure(23): n / ax / u (x axis linked, Hampel filter applied on n)
%
% Sam Jaeger
% jaege246@umn.edu
% 10/29/2025
%   Revised: 7/6/2026

function flight_test_plots(data_int)
    
    % 3d position
    figure(101)
    plot3(hampel(data_int.XYZ.n580(:,1)),hampel(-data_int.XYZ.n580(:,2)),hampel(-data_int.XYZ.n580(:,3)),'.')
    title('3D flight path','FontSize',20,'Interpreter','latex')
    xlabel('$X$ (ft)','FontSize',20,'Interpreter','latex')
    ylabel('$-Y$ (ft)','FontSize',20,'Interpreter','latex')
    zlabel('$-Z$ (ft)','FontSize',20,'Interpreter','latex')
    axis equal
    grid on

    % lat-long position plot from n580 solution
    figure(1001)
    geoplot(data_int.PHI_PSI_H.n580(:,1),data_int.PHI_PSI_H.n580(:,2),'.r')
    geobasemap satellite
    title('GPS Position','FontSize',16,'Interpreter','latex')
    geolimits([45.328 45.3293],[-93.2314 -93.23]) % ACRC Flying Field

    % u,v,w states from air data
    figure(10) 
    plot(data_int.t_out,data_int.uvw(:,1),'.' ,data_int.t_out,data_int.uvw(:,2),'.', data_int.t_out, data_int.uvw(:,3),'.'); 
    grid on; 
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('velocity (ft/s) - FMUR air data','FontSize',15,'Interpreter','latex'); 
    legend('u','v','w','location','eastoutside','Interpreter','latex')

    % angular velocity from n580
    figure(11); 
    plot(data_int.t_out, data_int.pqr.n580(:,1),'.', data_int.t_out, data_int.pqr.n580(:,2),'.', data_int.t_out, data_int.pqr.n580(:,3),'.'); 
    grid on; 
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('angular velocity (deg/s) - n580','FontSize',15,'Interpreter','latex'); 
    legend('$p$','$q$','$r$','location','eastoutside','Interpreter','latex')
    
    % attitude from n580
    figure(12); 
    hb(1)=subplot(3,1,1);
    plot(data_int.t_out,data_int.phi,'.'); 
    grid on; 
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$\phi$ (deg) - n580','FontSize',15,'Interpreter','latex'); 
    hb(2)=subplot(3,1,2);
    plot(data_int.t_out,data_int.theta,'.');
    grid on
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$\theta$ (deg) - n580','FontSize',15,'Interpreter','latex'); 
    hb(3)=subplot(3,1,3);
    plot(data_int.t_out,data_int.psi,'.'); 
    grid on
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$\psi$ (deg) - n580','FontSize',15,'Interpreter','latex'); 
    linkaxes(hb,'x')

    % acceleration from n580
    figure(13); 
    plot(data_int.t_out, data_int.ax.n580,'.', data_int.t_out, data_int.ay.n580,'.', data_int.t_out, data_int.az.n580,'.'); 
    grid on; 
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('acceleration $(ft/s^2)$ - n580','FontSize',15,'Interpreter','latex'); 
    legend('$x$','$y$','$z$','location','eastoutside','Interpreter','latex')

    % alpha beta from probes
    figure(14)
    plot(data_int.t_out, data_int.alpha,'.', data_int.t_out, data_int.beta,'.', data_int.t_out, data_int.beta_f,'.')
    xlabel('time (s)','FontSize',20,'Interpreter','latex')
    ylabel(' $\alpha$, $\beta$, $\beta_f$ (deg)','FontSize',20, 'Interpreter','latex')
    grid on
    legend(' $\alpha$',' $\beta$',' $\beta_f$','FontSize',20, 'Interpreter','latex','Location','southeast')

    % control surface time histories
    figure(15);
    title('Control Time history')
    hc(1)=subplot(4,1,1);
    plot(data_int.t_out, data_int.rpm,'.r'); ylim([0 12000])
    xlabel('time (s)','FontSize',20,'Interpreter','latex')
    ylabel('$rpm$','FontSize',20,'Interpreter','latex')
    grid on
    hc(2)=subplot(4,1,2);
    plot(data_int.t_out, data_int.delta_e,'.r'); ylim([-30 30])
    xlabel('time (s)','FontSize',20,'Interpreter','latex')
    ylabel('$\delta_e$ (deg)','FontSize',20,'Interpreter','latex')
    grid on
    hc(3)=subplot(4,1,3);
    plot(data_int.t_out, data_int.delta_a_r,'.r', data_int.t_out, data_int.delta_a_l,'.b', data_int.t_out, data_int.delta_a,'.g'); ylim([-30 30])
    xlabel('time (s)','FontSize',20,'Interpreter','latex')
    ylabel('$\delta_a$ (deg)','FontSize',20,'Interpreter','latex')
    legend('right','left','avg')
    grid on
    hc(4)=subplot(4,1,4);
    plot(data_int.t_out, data_int.delta_r,'.r'); ylim([-30 30])
    xlabel('time (s)','FontSize',20,'Interpreter','latex')
    ylabel('$\delta_r$ (deg)','FontSize',20,'Interpreter','latex')
    grid on
    linkaxes(hc,'x')




    % custom plots for Sys ID ---------------------------------------------

    % alpha / q / delta_e
    figure(20); 
    ha(1)=subplot(3,1,1);
    plot(data_int.t_out,data_int.alpha,'.'); 
    grid on; 
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$\alpha$ (deg)','FontSize',15,'Interpreter','latex'); 
    ha(2)=subplot(3,1,2);
    plot(data_int.t_out,data_int.pqr.n580(:,2),'.');
    grid on
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$q$ $(\deg/s)$ - n580','FontSize',15,'Interpreter','latex'); 
    ha(3)=subplot(3,1,3);
    plot(data_int.t_out,data_int.delta_e,'.'); 
    grid on
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$\delta_e$ (deg)','FontSize',15,'Interpreter','latex'); 
    linkaxes(ha,'x')

    % beta / p / delta_a
    figure(21); 
    hd(1)=subplot(3,1,1);
    plot(data_int.t_out,data_int.beta,'.'); 
    grid on; 
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$\beta$ (deg)','FontSize',15,'Interpreter','latex'); 
    hd(2)=subplot(3,1,2);
    plot(data_int.t_out,data_int.pqr.n580(:,1),'.');
    grid on
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$p$ $(\deg/s)$ - n580','FontSize',15,'Interpreter','latex'); 
    hd(3)=subplot(3,1,3);
    plot(data_int.t_out,data_int.delta_a,'.'); 
    grid on
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$\delta_a$ (deg)','FontSize',15,'Interpreter','latex'); 
    linkaxes(hd,'x')

    % beta / r / delta_r
    figure(22); 
    he(1)=subplot(3,1,1);
    plot(data_int.t_out,data_int.beta,'.'); 
    grid on; 
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$\beta$ (deg)','FontSize',15,'Interpreter','latex'); 
    he(2)=subplot(3,1,2);
    plot(data_int.t_out,data_int.pqr.n580(:,3),'.');
    grid on
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$r$ $(\deg/s)$ - n580','FontSize',15,'Interpreter','latex'); 
    he(3)=subplot(3,1,3);
    plot(data_int.t_out,data_int.delta_r,'.'); 
    grid on
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$\delta_r$ (deg)','FontSize',15,'Interpreter','latex'); 
    linkaxes(he,'x')

    % n / ax / u
    figure(23); 
    hf(1)=subplot(3,1,1);
    plot(data_int.t_out,hampel(data_int.n),'.'); 
    grid on; 
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$n$ (rev/s)','FontSize',15,'Interpreter','latex'); 
    hf(2)=subplot(3,1,2);
    plot(data_int.t_out,data_int.ax.n580,'.');
    grid on
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$a_x$ $(ft/s^2)$ - n580','FontSize',15,'Interpreter','latex'); 
    hf(3)=subplot(3,1,3);
    plot(data_int.t_out,data_int.uvw(:,1),'.'); 
    grid on
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$u$ (ft/s)','FontSize',15,'Interpreter','latex'); 
    linkaxes(hf,'x')
end