%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MODEL PREDICTIVE CONTROL: AUTONOMOUS CARS WITHOUT COMUNICATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% We assume that two vehicles cannot communicate.
% In this problem, a vehicle as the Leader follows a path that is generated
% by SPP algorithm. The follower observes the location of the Leader by its
% sensors with noise and generates its own path based on TTC (Time to Collision)
% and obtained data by measured . The objective function for the leader is: 
% fuel consumption, arrival time to the goal point, deviation from the reference path.
% the objective for the Follower is: 
% fuel consumption, safety, distance from the leader location.

% this version coniders just a simple dynamic model of mass to speed up the
% algorithm. the reasn that we designed this version was running time. the
% running time of MPC in version 6 is more than 100 min and it confines us
% to reach a compariosn we are looking for. 

% the former (original) file is: "L_F_without_communication_5.m"
% the next file is: "" ==> goal:
% working on this file started  : 2/14/2018
% working on this file Finished : 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function L_F_without_communication_6_massDynamicModel
    
    
    clc
    clear all
    clearvars
    clf
    rng(1984)
    
    global mpciterations dt xg xr alpha1_L alpha2_L alpha3_L alpha4_L ...
           TTC_min safety_distance big_M meduim_M alpha1_F alpha2_F alpha3_F alpha4_F alpha5_F ...
           m_L T_m_L omega_m_L beta_L alpha_n_L c_r_L c_d_L A_L a_L b_L dDelta_max_L v_max_L steering_ang_max_L...
           m_F T_m_F omega_m_F beta_F alpha_n_F c_r_F c_d_F A_F a_F b_F dDelta_max_F v_max_F steering_ang_max_F...
           theta_s rho g mu_road Confidence_Level virtual_lead_line applied_steps braking_coefficient critical_distance

       
    mpciterations = 115;       % Number of MPC iterations to be performed
    N = 15;                    % Length of Optimization Horizon
    dt = 0.4;                  % time step
    applied_steps=4;
    big_M=1e100; % big_M is a big number
    meduim_M=8;  % meduim_M is for linearizing 16~32.
    % it is the number of line segements used to approximate a circle 
    Confidence_Level=0.95;% Confidence_Level shows the probability of 
    % achieving to (happening) a desirable solution and situation (1-alpha)=Confidence_Level
    plot_resolution=1;% the bigger this number is, the more precise the plot is
    % plot_resolution=1 shows the real output of the MPC
    % plot_animation=1;% 1 is "YES", 0 is "NO"
    wait_plot=0; % this time is the gap to plot the next location (Leader and Follower)
    tracking=1;
    virtual_lead_line=[10  10  12  120
                       40  125 127 127];% this virtual line defines street. we can use it 
                   % to define a street. by keeping the amount of deviation
                   % from this line a vehicle can avoid the constraints
                   % around the street
   braking_coefficient=6; % if vehicle pushes on brake it will be 
   %improved braking_coefficient=3 times 
%%%%%%  LEADER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % STATE=  [x   ; y   ; dx   ; dy  ]; units=[m m rad m/s m/s rad/s rad]    
    x0_L   =  [10   ; 55  ; 0    ; 0  ]; % REAL value for x0_L
    % it is notable that the variance matrix is constant and does
    % not grow with time
    %[x_L(1); x_L(2); x_L(3); x_L(4); x_L(5); x_L(6); x_L(7)] Initial State
    % of the Leader x(1)=x_L; x(2)=y_L; x(3)=theta_L (heading angle) ; x(4)=dx_L ;
    % x(5)=dy_L; x(6)=dTheta_L (heading angular velocity); x(7)=delta_L (steering angle)
    % [x-coord_L; y-coord_L; heading_L;dx-coord_L; dy-coord_L; dHeading_L; steering_L]
    % position_x, position_y, heading_theta, velocity_x, velocity_y, and dHeading  

%     xr= [10    10   30   
%          100   127  127];
     
     xr= [10  10   12   80   
          40  125  127  127];%reference path which Leader should follow
    xg= [xr(1,end) xr(2,end) pi/2]; % goal position for the Leader (last point of xr and desired heading angle)

    % in format [x1 x2 , ... ; y1 y2 ...] this signal can be the output of SPP algorithm.

    % u0_L(1): (percent) the first variable shows the "position of throttle or brake" at x-coordinate which is [-1 1]
    % u0_L(2): this is released for this version .(rad/s) the second variable shows the "steering anglular velocity" which has a maximum value "stearingAngVel_max_L": [0 stearingAngVel_max_L]
    % u0_L(3): (percent) the third variable shows the "position of throttle or brake" at y-coordinate  which is [-1 1]
    
    alpha1_L=1; % cost coeificient. Distance of the Leader from the goal point at each time step 
    alpha2_L=0; % cost coeificient for "u_L(1)" as the Leader's input signal for 
    % the position of throttle(acceleration equivalent to the fuel consumption
    %, work on the real objective function). 
    alpha3_L=45; % cost coeificient for deviation of the Leader from the Reference path
    alpha4_L=0; % cost coeificient at the Leader's final stage (arrival to the goal point)
    x_L = zeros(length(x0_L),mpciterations+1);%Leader's expected state variables at each step
    x_L(:,1)=x0_L;% it puts the initial state (x0) as the current state (x_curr)

    u_L = zeros(2,mpciterations+1);%Leader's control signal at each step
    theta_s=0; %  slope of the road (rad)
    mu_road=1; %  Coefficient of road adhesion (1:Good, dry ; 0.9:Good,wet ;0.8:Poor,dry ;0.6:Poor, wet ;0.25: Packed snow or ice)
    rho=1.3; % density of air (kg/m^3)
    g=9.8; % gravitational constant (m/s*s)
    
    % Vehicle characteristics of the Leader
    v_max_L=6; % maximum velocity of the Leader (m/s)
    m_L= 1000;% mass of the Leader (kg)
    T_m_L=190; % maximum torque of the Leader's engine (Nm)
    omega_m_L=420; % maximum engine speed of the Leader (rad/s)
    beta_L=0.4; % constant coeficient in Leader's engine
    alpha_n_L=30; % alpha_n_L=n/r (n: gear ratio; r: wheel radius) in Leader
    c_r_L=0.01; % coefficient of rolling friction for Leader
    c_d_L=0.32; % shape-dependent aerodynamic drag coefficient for Leader
    A_L=2.4; % the frontal area of the Leader (m^2)
    a_L= 1.2; % distance between the rear wheels and center of the mass in the Leader (m)
    b_L= 2; % distance between the rear and front wheels in the Leader(m)
    dDelta_max_L=pi/6; % maximum value for the rate of changes in Leader's steering angle (rad/s)
    steering_ang_max_L=0.4363; % (rad) the maximum steering angle of the Leader (25 degree)
    
    % initial solution for the Leader optimization problem
    u0_L(1,:)=0.5*ones(1,N);% throttle
    %u0_L(2,:)=0 + (0.1*dDelta_max_L-0).*ones(1,N);% rate of changes in streeing angle:                    
    u0_L(2,:)=0.5*ones(1,N); % brake    %a + (b-a).*rand(100,1) 100 random number between [a,b]

%%%%%%  FOLLOWER  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
    TTC_min=0.3; % minimum amount of TTC(Time-to-Collision) (s)
    safety_distance=15;% minimum distance between the Leader and Follower (their distance should be bigger than this amount)
    critical_distance=0.6*safety_distance; % vehicle should not enter into critical distance which is 60% of safety distance
    % Initial State
    %             [ x_F(1); x_F(2); x_F(3)    ; x_F(4); x_F(5); x_F(6)    ; x_F(7)    ] 
    % STATE=      [ x     ; y     ; theta     ; dx    ; dy    ; d_theta   ; delta     ]; % units=[m m rad m/s m/s rad/s rad]    
    x0_F(:,1,1)=  [ 10    ; 20     ; 0     ; 0    ]; % expected value for x0_F measured by GPS and Follower vehicle itself
    x0_F(:,1,2)=  [ 0.03  ; 0.03  ; 0.02  ; 0.02  ]; % variance of x0_F measured by GPS and Follower itself
    % x0_F(:,1,1): (mu)  shows the expected values
    % x0_F(:,1,2): (var) shows teh variances of the variables
    % STATE=              [x   ; y   ; theta    ; dx   ; dy   ; d_theta   ; delta    ]; % units=[m m rad m/s m/s rad/s rad]    
    x0_L_measuredByF(:,1,1)=  x0_L               ; % expected value for x0_L measured by the Follower
    x0_L_measuredByF(:,1,2)= [0.05; 0.05; 0.03 ; 0.03]; % variance of x0_L measured by Follower    
    % x0_L_measuredByF(:,1,1): (mu)  shows the "expected values" of the Leader's state variables measured by the Follower
    % x0_L_measuredByF(:,1,2): (var) shows the       "variances" of the Leader's state variables measured by the Follower
    % Note: the Follower can measure the (x,y) and (dx,dy) for the Leader but it cannont measure its theta,d_theta and delta. So, 
    % to apply this fact we put the max possible amount for their variances
    % But, this means that we won't use them in our calculations and its
    % just a symbol.
    % of the Follower x(1)=x_F; x(2)=y_F; x(3)=theta_F (heading angle); x(4)=dx_F;
    % x(5)=dy_F; x(6)=dTheta_F; x(7)=delta_F (steering angle)
    % [x-coord_F; y-coord_F; heading_F;dx-coord_F; dy-coord_F; dHeading_F; steering_F]
    % position_x, position_y, heading_theta, velocity_x, velocity_y, and dHeading
    % u0_F(1): (percent) the first variable shows the "position of throttle"  which is [0 1]
    % u0_F(2): (rad/s) the second variable shows the "heading angular velocity" which has a
    % maximum value "stearingAngVel_max_F": [0 stearingAngVel_max_F]
    % u0_F(3): (percent) the third variable shows the "position of brake"  which is [0 1]
    
    % Follower's cost coeficient in objective function
    alpha1_F=1; % cost coeificient for the Distance from the Leader's position (as a moving goal point) at each time step 
    alpha2_F=100; % cost coeificient for fuel consumption
    %, work on the real objective function).
    alpha3_F=0; % cost coeificient for deviation from "TTC_min"
    alpha4_F=0; % cost coeificient for the minimum distance "safety_distance"
    alpha5_F=5; % cost coeificient for the deviation of the Follower from the Virtal Lead Line
    x_F=zeros(length(x0_F(:,1,1)),mpciterations+1,2);% the 1st page show "mu", and the 2nd page shows "var"
    x_F(:,1,1)=x0_F(:,1,1); % substitute the mu  of the initial state of the Follower
    x_F(:,1,2)=x0_F(:,1,2); % substitute the var of the initial state of the Follower
    u_F = zeros(2,mpciterations+1);%Follower's control signal at each step
    % we do not consider uncertainty in our control values i.e. "u". We assume that all these uncertainties 
    % will be summurized at the state variables i.e.
    % x,y,dx,dy,theta,d_theta, delta
    % Vehicle characteristics of the Follower
    v_max_F=6; % maximum velocity of the Follower (m/s)
    m_F= 1000;% mass of the Follower (kg)
    T_m_F=190; % maximum torque of the Follower's engine (Nm)
    omega_m_F=420; % maximum engine speed of the Follower (rad/s)
    beta_F=0.4; % constant coeficient in Follower's engine
    alpha_n_F=25; % alpha_n_L=n/r (n: gear ratio; r: wheel radius) in Follower
    c_r_F=0.01; % coefficient of rolling friction for Follower
    c_d_F=0.32; % shape-dependent aerodynamic drag coefficient for Follower
    A_F=2.4; % the frontal area of the Follower (m^2)
    a_F= 1.2; % distance between the rear wheels of the Follower and its center of the mass (m)
    b_F= 2; % distance between the rear and front wheels of the Follower(m)
    dDelta_max_F=pi/6; % maximum value for the rate of changes in Follower's steering angle (rad/s)
    steering_ang_max_F=0.4363; % (rad) the maximum steering angle of the Follower (25 degree)
    
    % initial solution for the Follower optimization problem
    u0_F(1,:)=1*ones(1,N);% throttle
    %u0_F(2,:)=0 + (0.1*dDelta_max_F-0).*ones(1,N);% rate of changes in steering angle
    u0_F(2,:)=0.55*ones(1,N); % brake
    %0.6 0.7 0.8 0.9 1
    %(1,0.5) best answer till now
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    t0= 1;                       % starting time
    t = zeros(1,mpciterations+1);% time steps
    t_curr=t0;% it puts the zero time (t0) as the current time step (t_curr)

    tic
    objective_L=zeros(mpciterations,1);
    objective_F=zeros(mpciterations,1);
    
    for i=2:applied_steps:mpciterations+1

    %%%%%% Leader %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        [u_new_L,obj_L] = OptimalControl_L(@runningcosts_L, @terminalcost_L, @constraints_L, ...
                               @terminalconstraints_L, @linearconstraints_L, ...
                               @SystemModel_L, N, t_curr, x_L(:,i-1), u0_L, dt);
                           
        
       %{
       % OptimalControl: this function gets some inputs and returns the
       % optimal value for the input signal (u) which is decision variable
       % @runningcosts:
       % @terminalcost:
       % @constraints:
       % @terminalconstraints:
       % @linearconstraints: 
       % @SystemModel: returns the state of the model based on the given
       %    input/decision varaiable u. (u is unknown and "@" shows that SystemModel
       %    is in parametric format). e.g. x(3)= 2*u(1)+0.5*dt^2
       % N: Length of the Optimization Horizon
       % t_curr: current time step
       % x_curr: current state
       % u0: initial solution/value for input/control signal
       % dt: length of time step
       %}                  
       
        u_L(:,i:i+applied_steps-1)=u_new_L(:,1:applied_steps); %save the results in matrix u_L
        objective_L(i:i+applied_steps-1,1)=obj_L;
        
        % apply control on the Leader
        % in this phase, we just apply the fisrt step and do the
        % optimization again. this can be increased.
        
        for counter=1:applied_steps
            x_L(:,i-1+counter) = SystemModel_L(t_curr, x_L(:,i-1+counter-1), u_L(:,i-1+counter),dt); % apply control for as many as applied_steps
        end
        
        % x_curr_L = SystemModel_L(t_curr, x_curr_L, u_new_L,dt); % apply control
        %{
        % SystemModel: this function gets the inputs and returns the state
        % as the outcome
        % t_curr: current time
        % x_curr_L: current state of vehicle
        % u_L(:,i): the control command
        % dt: time step to apply the control command
        %}
    %%%%%% Follower %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Find the optimal control command for the Follower

        x_curr_L_measuredByF(:,1,1)=x_L(:,i+applied_steps-1);% mu of the current state of the Leader that has been measured by the Follower
        x_curr_L_measuredByF(:,1,2)=x0_L_measuredByF(:,1,2); % var of the current state of the Leader that has been measured by the Follower
        [u_new_F,obj_F] = OptimalControl_F(@runningcosts_F, @terminalcost_F, @constraints_F, ...
                               @terminalconstraints_F, @linearconstraints_F, ...
                               @SystemModel_F, N, t_curr, x_F(:,i-1,:), u0_F, dt, x_curr_L_measuredByF); 
        
                         
        objective_F(i:i+applied_steps-1,1)=obj_F;                   
        u_F(:,i:i+applied_steps-1)=u_new_F(:,1:applied_steps); %save the results in matrix u_F                          

        % apply control on the Follower
        % apply control to find "mu" of the current state of the Follower  
        for counter=1:applied_steps
            x_F(:,i-1+counter,:) = SystemModel_F(t_curr, x_F(:,i-1+counter-1,:), u_F(:,i-1+counter),dt); % apply control for as many as applied_steps
        end
        % x_curr_F(:,1,1): "mu"  of the current state of the Follower
        % x_curr_F(:,1,2): "var" of the current state of the Follower which stays same as its initial values
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        t0 = t_curr+dt; % calculates the next time step
        t(:,i)=t0; % save the next time step in time step matrix
%         x_L(:,i)=x_curr_L; % save the current state of the Leader after applying the control in state matrix 
%         x_F(:,i,1)=x_curr_F(:,1,1); % save the "mu"  of the current state of the Follower after applying the control in state matrix 
%         x_F(:,i,2)=x_curr_F(:,1,2); % save the "var" of the current state of the Follower after applying the control in state matrix 
%     
        progress=100*(i-1)/mpciterations;
        fprintf('%4.2f\n',progress)
    end

%%%%%% Print the Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    filename='MyFile_6_massDynamicModel.txt';
    fid=fopen(filename,'w');
    fprintf(fid,'-------------------------------------------- LEADER --------------------------------------\n');
    fprintf(fid,'iteration         u1          u2           dx           dy           v\n');
    fprintf(fid,'------------------------------------------------------------------------------------------\n');

    overall_velocity_L=zeros(mpciterations,1);
    for i=2:mpciterations+1
 
        overall_velocity_L(i)=sqrt((x_L(3,i))^2+(x_L(4,i))^2);
        fprintf(fid,' %3d     \t\t%+4.2f    \t%+4.2f     \t\t%+4.2f     \t%+4.2f     \t%+4.2f\n'...
                    ,  i,      u_L(1,i),     u_L(2,i),    x_L(3,i),      x_L(4,i),    overall_velocity_L(i));

    end    
       
    fprintf(fid,'\n\n------------------------------------------- FOLLOWER -------------------------------------\n');
    fprintf(fid,'iteration         u1          u2           dx           dy           v\n');
    fprintf(fid,'------------------------------------------------------------------------------------------\n');
    
    overall_velocity_F=zeros(mpciterations,1);
    for i=2:mpciterations+1
        overall_velocity_F(i)=sqrt((x_F(3,i,1))^2+(x_F(4,i,1))^2);
        fprintf(fid,' %3d     \t\t%+4.2f    \t%+4.2f      \t\t%+4.2f     \t%+4.2f     \t%+4.2f\n'...
                    ,  i,      u_F(1,i),     u_F(2,i),    x_F(3,i,1),    x_F(4,i,1),  overall_velocity_F(i));
    end
    fclose(fid);
    toc
%%%%%% Plot the Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    hold on
    grid on
    grid minor
    axis square
    axis([0 97 40 137])
    title('Vehicles WITHOUT Communication')% title
    xlabel('X') % x-axis label
    ylabel('Y') % y-axis label
    %legend('Reference Path','Leader','Follower','Location','northwest')% legend
    
    
    x_L_plot=zeros(length(x0_L),mpciterations*plot_resolution);% Leader's info to plot
    x_L_plot(:,1)=x0_L;% the initial state of the Leader
    x_F_plot=zeros(length(x0_F(:,1,1)),mpciterations*plot_resolution,2);% Follower's info to plot
    x_F_plot(:,1,:)=x0_F(:,1,:);% the initial state of the Follower
    t_curr_plot =0;
    
    for i=1:mpciterations+1
        for j=1:plot_resolution
            if (i+j>2)% for i+j=2 it show the intital point which is given
                x_L_plot(:,(i-1)*plot_resolution+j)=...
                    SystemModel_L(t_curr_plot, x_L_plot(:,(i-1)*plot_resolution+j-1)  , u_L(:,i),dt/plot_resolution);

                x_F_plot(:,(i-1)*plot_resolution+j,:)=...
                    SystemModel_F(t_curr_plot, x_F_plot(:,(i-1)*plot_resolution+j-1,:), u_F(:,i),dt/plot_resolution);
                    
                t_curr_plot = t_curr_plot+dt/plot_resolution;
            end
        end
    end
    
    x_F_plot(:,:,2)=[]; % delete the 2nd page that shows the variance. we don't need it for ploting
    
    
    vidfile = VideoWriter('L_F_without_communication_6_massDynamicModel.mp4','MPEG-4');
    vidfile.FrameRate = 5; % How many frames per second.
    open(vidfile); 
    
    subplot(3,2,[1 3])
    hold on
    title('Vehicles Without Communication')
    plot(xr(1,:),xr(2,:),'--'); % plot the position of reference path (desired)
    enviroment_plot;% plot the environmet including the street and areas

    h101=cell(1,1);
    h102=cell(1,1);
    for i=1:(mpciterations+1)*plot_resolution

        if tracking==0
            for j=1:1
                delete(h101{j,1});
                delete(h102{j,1});  
            end
        end
        h101=car_plot_L(x_L_plot(:,i));
        h102=car_plot_F(x_F_plot(:,i));
        drawnow;
        Movie_maker= getframe(gcf);
        pause(wait_plot)
        writeVideo(vidfile, Movie_maker);
    end
% close(vidfile); % Saves the movie.

draw_diagram(u_L,x_L,overall_velocity_L,objective_L,u_F,x_F,overall_velocity_F,objective_F)

end

%%  LEADER'S FUNCTIONS
function [u_new_L,v_cost_L]=OptimalControl_L(runningcosts_L, terminalcosts_L, ...
                             constraints_L, terminalconstraints_L, ...
                             linearconstraints_L, SystemModel_L, N, t_curr, x_curr_L, u0_L, dt)
     
    %x_L = zeros(N+1, length(x_curr_L));
    x_L = computeOpenloopSolution_L(SystemModel_L, N, dt, t_curr, x_curr_L, u0_L);                     
    % Set control and linear bounds
    A_L = [];
    b_L = [];
    Aeq_L = [];
    beq_L = [];
    lb_L = [];
    ub_L = [];

    for k=1:N

        [Anew_L, bnew_L, Aeqnew_L, beqnew_L, lbnew_L, ubnew_L] = ...
               linearconstraints_L(t_curr+(k-1)*dt,x_L(:,k),u0_L(:,k));
        A_L = blkdiag(A_L,Anew_L);
        b_L = [b_L, bnew_L];
        Aeq_L = blkdiag(Aeq_L,Aeqnew_L);
        beq_L = [beq_L, beqnew_L];
        lb_L = [lb_L, lbnew_L];
        ub_L = [ub_L, ubnew_L];
    end                     
     %options= optimset('Display','off','MaxIterations',100);

    % options= optimoptions('Display','off','MaxIterations',100);
     options = optimoptions('fmincon','Algorithm','interior-point');

     
    % ,...         'MaxFunctionEvaluations',5000,'MaxIterations',1000);
     
    [u_new_L, v_cost_L] = fmincon(@(u_L) costfunction_L(runningcosts_L,...
        terminalcosts_L, SystemModel_L, N, dt, t_curr, x_curr_L, u_L)...
    , u0_L, A_L, b_L, Aeq_L, beq_L,lb_L, ub_L, ...
    @(u_L) nonlinearconstraints_L(constraints_L, terminalconstraints_L,...
    SystemModel_L, N, dt, t_curr, x_curr_L, u_L),options);                     
                         
end

function x_L = computeOpenloopSolution_L(SystemModel_L, N, dt, t_curr, x_curr_L, u_L)
    x_L(:,1) = x_curr_L;
    for k=1:N
        x_L(:,k+1) = SystemModel_L(t_curr, x_L(:,k), u_L(:,k), dt);
    end
end

function cost_L = costfunction_L(runningcosts_L, terminalcosts_L, SystemModel_L, ...
                    N, dt, t_curr, x_curr_L, u_L)
    cost_L = 0;
    %x_L = zeros(length(x_curr_L), N+1);
    x_L = computeOpenloopSolution_L(SystemModel_L, N, dt, t_curr, x_curr_L, u_L);
    for k=1:N
        cost_L = cost_L+runningcosts_L(t_curr+(k-1)*dt, x_L(:,k), u_L(:,k));
    end
    cost_L = cost_L+terminalcosts_L(t_curr+(N+1)*dt, x_L(:,N+1));
end

function [c_L,ceq_L] = nonlinearconstraints_L(constraints_L, ...
    terminalconstraints_L, SystemModel_L, N, dt, t_curr, x_curr_L, u_L)
    %x_L = zeros(N+1, length(x_curr_L));
    x_L = computeOpenloopSolution_L(SystemModel_L, N, dt, t_curr, x_curr_L, u_L);
    c_L = [];
    ceq_L = [];
    for k=1:N
        [cnew_L, ceqnew_L] = constraints_L(t_curr+(k-1)*dt,x_L(:,k),u_L(:,k));
        c_L = [c_L cnew_L];
        ceq_L = [ceq_L ceqnew_L];
    end
    [cnew_L, ceqnew_L] = terminalconstraints_L(t_curr+(N+1)*dt,x_L(:,N+1));
    c_L = [c_L cnew_L];
    ceq_L = [ceq_L ceqnew_L];
end


function cost_L = runningcosts_L(t,x_L, u_L)

    global xg alpha1_L alpha3_L xr %alpha2_L  
    
    distance=point_to_line(x_L(1:2),xr);
    cost_L = alpha1_L*(abs((x_L(1)-xg(1)))+abs((x_L(2)-xg(2))))+...
                                                        alpha3_L*distance;
         %sqrt(((x_L(1)-xg(1)))^2+((x_L(2)-xg(2)))^2)+...
    %alpha2_L*u_L(1);
         
             
end

function tcost_L=terminalcost_L(t,x_L)
    %global xg alpha4_L
    tcost_L=0;%alpha4_L*(sqrt((x_L(1)-xg(1))^2+(x_L(2)-xg(2))^2));%+ abs(x_L(3)-xg(3))
end

function [c_L,ceq_L] = constraints_L(t,x_L, u_L)
    global v_max_L steering_ang_max_L xr big_M mpciterations xg meduim_M
    %distance=point_to_line(x_L(1:2),xr);
    c_L(1)   = (x_L(3)^2+x_L(4)^2-(v_max_L)^2);%[]; this means that the velocity of the vehicle can not be greater than v_max
    %c_L(2)   = (x_L(7)-steering_ang_max_L);%x_L(7)<=steering_ang_max_L
    
%     for i=1:meduim_M
%         
%         c_L(i)=x_L(3)*cos(2*pi*i/meduim_M)+x_L(4)*sin(2*pi*i/meduim_M)-v_max_L;
%         
%     end
    
    %ceq_L(1) = (u_L(1)*u_L(3)-0);%[]; % this means that just one of the throttle or brake can be applied at a single moment
     
%     % obstacle 1 (area 1)
%     c_L(3)  = -x_L(1)+8;
%     c_L(4)  =  x_L(2)-129; 
%     
%     % obstacle 2 (area 2)
%     %the following line shows "outside" area of a rectangle with center
%     %(74.5,62.5), length (125) and width (125). 
%     c_L(5)  =  1-abs(((x_L(1)-74.5)/125)+((x_L(2)-62.5)/125))-abs(((x_L(1)-74.5)/125)-((x_L(2)-62.5)/125));
    
    % obstacle avoidance constraints. Pr(collision)< 1-CL
    exchange_5=length(c_L);
    if (x_L(2)<125)
          c_L(exchange_5+1) = 8-x_L(1);
          c_L(exchange_5+2) = x_L(1)-12;
    else
          c_L(exchange_5+1) = 8-x_L(1);
          c_L(exchange_5+2) = x_L(2)-129;
    end

    if (x_L(1)>12)
          c_L(exchange_5+3) = 125-x_L(2);
          c_L(exchange_5+4) = x_L(2)-129;
    else
          c_L(exchange_5+3) = 8-x_L(1);
          c_L(exchange_5+4) = x_L(2)-129;
    end
    
    
    ceq_L=[];
    

end

function [c_L,ceq_L] = terminalconstraints_L(t,x_L)
    global mpciterations dt xg
    c_L   = [];
    ceq_L = [];
end

function [A_L, b_L, Aeq_L, beq_L, lb_L, ub_L] = linearconstraints_L(t,x_L, u_L)

    global dDelta_max_L 
    A_L   = []; % a1*u_L(1) + a2*u_L(2) + a3*u_L(3)  <= 5  ==> A_L = [a1 a2 a3] and b_L=[5]
    b_L   = []; %
    Aeq_L = [];
    beq_L = []; 
    
    % throttle position [0 1] ; changes in steering angle [0 stearingAngVel_max_L]
    lb_L  = [-1  -1];%  -.001  -.001]; % u_L(1)>=0, u_L(2)>= -dDelta_max_L, u_L(1)>=0
    ub_L  = [1    1];%   1.001 1.001]; % u_L(1)<=1, u_L(2) <= dDelta_max_L, u_L(3)<=1
end

function x_L = SystemModel_L(t,x_L, u_L, dt)
    % this function gets the inputs and returns the state x
    % t: current time
    % x_L: the current state of the vehicle
    % u_L: the control signal
    % dt: time step
 
    global alpha_n_L T_m_L beta_L omega_m_L m_L theta_s g rho c_d_L A_L c_r_L a_L b_L mu_road braking_coefficient
    
    
    
    if (u_L(1)<0)
        u_L(1)= braking_coefficient*u_L(1);
    elseif ((u_L(2)<0))
        u_L(2)=braking_coefficient*u_L(2);
    end
    
    
    % simple dynamic model for vehicle ref: https://github.com/NikolasEnt/Model-Predictive-Control
    x_L(3)= x_L(3) + u_L(1)*dt; % velocity of the Leader at x-coordinate
    x_L(4)= x_L(4) + u_L(2)*dt; % velocity of the Leader at y-coordinate 
       
    x_L(1)=x_L(1)+dt*x_L(3); % x-coordinate of the Leader (new)
    x_L(2)=x_L(2)+dt*x_L(4); % y-coordinate of the Leader (new)

end

%%  FOLLOWER'S FUNCTIONS

function [u_new_F,v_cost_F]=OptimalControl_F(runningcosts_F, terminalcosts_F, ...
                             constraints_F, terminalconstraints_F, ...
                             linearconstraints_F, SystemModel_F, N, t_curr, x_curr_F, u0_F, dt, x_curr_L_measuredByF)
     
    %x_F = zeros(N+1, length(x_curr_F));
    x_F = computeOpenloopSolution_F(SystemModel_F, N, dt, t_curr, x_curr_F, u0_F);                     
    % Set control and linear bounds
    A_F = [];
    b_F = [];
    Aeq_F = [];
    beq_F = [];
    lb_F = [];
    ub_F = [];
    for k=1:N
        [Anew_F, bnew_F, Aeqnew_F, beqnew_F, lbnew_F, ubnew_F] = ...
               linearconstraints_F(t_curr+(k-1)*dt,x_F(:,k,:),u0_F(:,k), x_curr_L_measuredByF);
        A_F = blkdiag(A_F,Anew_F);
        b_F = [b_F, bnew_F];
        Aeq_F = blkdiag(Aeq_F,Aeqnew_F);
        beq_F = [beq_F, beqnew_F];
        lb_F = [lb_F, lbnew_F];
        ub_F = [ub_F, ubnew_F];
    end    
    %2500---137s
    %10000--131
    %100 -- 132
     options= optimset('Display','off','Algorithm','interior-point');     
     
    [u_new_F, v_cost_F] = fmincon(@(u_F) costfunction_F(runningcosts_F,...
        terminalcosts_F, SystemModel_F, N, dt, t_curr, x_curr_F, u_F, x_curr_L_measuredByF)...
    , u0_F, A_F, b_F, Aeq_F, beq_F,lb_F, ub_F, ...
    @(u_F) nonlinearconstraints_F(constraints_F, terminalconstraints_F,...
    SystemModel_F, N, dt, t_curr, x_curr_F, u_F, x_curr_L_measuredByF),options);
                        
                         
end

function x_F = computeOpenloopSolution_F(SystemModel_F, N, dt, t_curr, x_curr_F, u_F)
    x_F(:,1,:) = x_curr_F;
    for k=1:N
        
        x_F(:,k+1,:) = SystemModel_F(t_curr, x_F(:,k,:), u_F(:,k), dt);

    end
end

function cost_F = costfunction_F(runningcosts_F, terminalcosts_F, SystemModel_F, ...
                    N, dt, t_curr, x_curr_F, u_F, x_curr_L_measuredByF)
    cost_F = 0;
    %x_F = zeros(N+1, length(x_curr_F));
    x_F = computeOpenloopSolution_F(SystemModel_F, N, dt, t_curr, x_curr_F, u_F);
    for k=1:N
        cost_F = cost_F+runningcosts_F(t_curr+(k-1)*dt, x_F(:,k,:), u_F(:,k), x_curr_L_measuredByF);
    end
    cost_F = cost_F+terminalcosts_F(t_curr+(N+1)*dt, x_F(:,N+1,:), x_curr_L_measuredByF);
end

function [c_F,ceq_F] = nonlinearconstraints_F(constraints_F, ...
    terminalconstraints_F, SystemModel_F, N, dt, t_curr, x_curr_F, u_F, x_curr_L_measuredByF)
    %x_F = zeros(N+1, length(x_curr_F));
    x_F = computeOpenloopSolution_F(SystemModel_F, N, dt, t_curr, x_curr_F, u_F);
    c_F = [];
    ceq_F = [];
    for k=1:N
        [cnew_F, ceqnew_F] = constraints_F(t_curr+(k-1)*dt,x_F(:,k,:),u_F(:,k), x_curr_L_measuredByF);
        c_F = [c_F cnew_F];
        ceq_F = [ceq_F ceqnew_F];
    end
    [cnew_F, ceqnew_F] = terminalconstraints_F(t_curr+(N+1)*dt,x_F(:,N+1,:), x_curr_L_measuredByF);
    c_F = [c_F cnew_F];
    ceq_F = [ceq_F ceqnew_F];
end


function cost_F = runningcosts_F(t,x_F, u_F, x_curr_L_measuredByF)

    global  critical_distance alpha1_F alpha4_F big_M TTC_min dt safety_distance virtual_lead_line alpha5_F alpha2_F %alpha3_F
    %{
    % time to collision
    ttc=(sqrt(((x_curr_L_measuredByF(1,1,1)-x_F(1,1,1)))^2+((x_curr_L_measuredByF(2,1,1)-x_F(2,1,1)))^2))/...
        (sqrt((x_curr_L_measuredByF(3,1,1))^2+(x_curr_L_measuredByF(4,1,1))^2)-sqrt((x_F(3,1,1))^2+(x_F(4,1,1))^2));
    
    % ttc<0 : velocity of the Follower is less than that of the Leader
    % ttc>0 : velocity of the Follower is bigger than that of the Leader
    % work on this part more! what is CCT? study the literatures about it.
    % the folllowing is just the rudimentary design for TTC
    if (ttc>0 && ttc<TTC_min)
        ttc_penaltyCost= 1/(ttc);% in this case the penalty should be applied
    else
        ttc_penaltyCost=0;
    end
    %{
    
    % if the distance between the Follower and the Leader is less than "safety_distance"
    % this function applies a penalty as big as "big_M" to the objective
    % function, by this it prevents the Follower to place within the iven
    % distance "safety_distance"
    %}
    
    currentDistance=sqrt(((x_curr_L_measuredByF(1,1,1)-x_F(1,1,1)))^2+((x_curr_L_measuredByF(2,1,1)-x_F(2,1,1)))^2);
    if (currentDistance<=safety_distance)
        safety_distance_penalty=big_M; 
    else
        safety_distance_penalty=0;
    end
    %}
    %{
    if (u_F>=0)
        fuel_consumption_F=norm(u_F);
    else
        fuel_consumption_F=0;
    end
    %}
    
    deviationF_VLL=point_to_line(x_F(1:2,1,1),virtual_lead_line);
   
    Manhattan_distance_F_L=abs(x_curr_L_measuredByF(1,1,1)-x_F(1,1,1)) + abs(x_curr_L_measuredByF(2,1,1)-x_F(2,1,1));
    %safety_distance_penalty=safety_distance/(Manhattan_distance_F_L-critical_distance);

    
    if (Manhattan_distance_F_L>safety_distance) 
        
        safety_distance_penalty=(Manhattan_distance_F_L-safety_distance)/alpha1_F;
    else
        safety_distance_penalty=-alpha1_F*(Manhattan_distance_F_L-safety_distance);
    end
    
    % fuel consumption model 
    % ref:
    % (https://vtechworks.lib.vt.edu/bitstream/handle/10919/36471/ETD.pdf?sequence=1)
    % Chapter 3; Regression Model III
    % s shows speed; a shows acceleration
    speed_F=sqrt((x_F(3,1,1))^2+(x_F(4,1,1))^2);
    acceleration_F=sqrt((u_F(1))^2+(u_F(2))^2);
    fuel_consumption_F=dt*(exp((10^-5)*((20+0.3*speed_F/2+0.04*(speed_F^2)/2))*(1+acceleration_F+acceleration_F^2))-1);
    
    
    cost_F= alpha2_F*(fuel_consumption_F) + safety_distance_penalty + alpha5_F*deviationF_VLL;
    
           
     %{
    alpha1_F*safety_distance_penalty+
    +...
        alpha4_F*(safety_distance_penalty);
   
           alpha2_F*(fuel_consumption_F)+...
           alpha3_F*(ttc_penaltyCost)+...
           ;
      %}      
end

function tcost_F=terminalcost_F(t,x_F,x_curr_L_measuredByF)
    global alpha5_F
    % this cost is zero for now
    tcost_F=0;%*alpha5_F*((norm(x_curr_L_measuredByF(1,1,1)-x_F(1,1,1))+norm(x_curr_L_measuredByF(2,1,1)-x_F(2,1,1))));
end

function [c_F,ceq_F] = constraints_F(t,x_F, u_F, x_curr_L_measuredByF)
    % this function applies nonlinear constraints
    global v_max_F steering_ang_max_F Confidence_Level safety_distance big_M meduim_M

    % this means that the velocity of the vehicle can not be greater than v_max
    c_F(1)   = (x_F(3,1,1)^2+x_F(4,1,1)^2-(v_max_F)^2);
    
%     for i=1:meduim_M
%         
%         c_F(i)=x_F(3,1,1)*cos(2*pi*i/meduim_M)+x_F(4,1,1)*sin(2*pi*i/meduim_M)-v_max_F;
%         
%     end
    
    
    % pr( velocity of the Follower < maximum velocity ) > Confidence_Level
    %{
    %}
%     for i=1:meduim_M
%            
%         c_F(i)=  x_F(3,1,1)*cos(2*pi*i/meduim_M) + x_F(4,1,1)*sin(2*pi*i/meduim_M)...
%             - v_max_F + sqrt(2*(x_F(3,1,2) * (cos(2*pi*i/meduim_M))^2 + x_F(4,1,2) * (sin(2*pi*i/meduim_M))^2)) * erfinv(2*Confidence_Level-1);
%     end    
    
    
    
    
    % pr(distance between two vehicles > safety_distance) > Confidence_Level
    exchange_1=length(c_F);
      
    for i=1:meduim_M
           
        c_F(exchange_1+i)=  (x_F(1,1,1)-x_curr_L_measuredByF(1,1,1))*cos(2*pi*i/meduim_M) + ...
                            (x_F(2,1,1)-x_curr_L_measuredByF(2,1,1))*sin(2*pi*i/meduim_M) - safety_distance...
               - sqrt(2*((x_F(1,1,2)+x_curr_L_measuredByF(1,1,2))*(cos(2*pi*i/meduim_M))^2 ...
                       + (x_F(2,1,2)+x_curr_L_measuredByF(2,1,2))*(sin(2*pi*i/meduim_M))^2))...
               * erfinv(2*Confidence_Level-1);
    end
    
    % meduim_M=1;
%     c_F(1+meduim_M)=sqrt((x_F(1,1,1)-x_curr_L_measuredByF(1,1,1))^2+...
%         (x_F(2,1,1)-x_curr_L_measuredByF(2,1,1))^2)-safety_distance;% this can be deleted
    
    
    % this means that just one of the throttle or brake can be applied at a single moment
    %c_F(2+meduim_M+1)  = (u_F(1)*u_F(3)-0.0001);

    exchange_2=length(c_F);
    % obstacle avoidance constraints. Pr(collision)< 1-CL
    if (x_F(2,1,1)<125)
          
%           c_F(exchange_2+1) = 8-x_F(1,1,1);
%           c_F(exchange_2+2) = x_F(1,1,1)-12;
        % pr(x_F(1,1,1)>8)>CL
        c_F(exchange_2+1) = 8-x_F(1,1,1)+ sqrt(2*x_F(1,1,2))*erfinv(2*Confidence_Level-1); 
        % pr(x_F(1,1,1)<12)>CL
        c_F(exchange_2+2) = x_F(1,1,1)-12+sqrt(2*x_F(1,1,2))*erfinv(2*Confidence_Level-1);
        
    else
        
%           c_F(exchange_2+1) = 8-x_F(1,1,1);
%           c_F(exchange_2+2) = x_F(2,1,1)-129;
        % pr(x_F(1,1,1)>8)>CL
        c_F(exchange_2+1) = 8-x_F(1,1,1)+sqrt(2*x_F(1,1,2))*erfinv(2*Confidence_Level-1);
        % pr(x_F(2,1,1)<129)>CL
        c_F(exchange_2+2) = x_F(2,1,1)-129+sqrt(2*x_F(2,1,2))*erfinv(2*Confidence_Level-1);

    end

    if (x_F(1,1,1)>12)
          
%           c_F(exchange_2+3) = 125-x_F(2,1,1);
%           c_F(exchange_2+4) = x_F(2,1,1)-129;
        % pr(x_F(2,1,1)>125)>CL
        c_F(exchange_2+3) = 125-x_F(2,1,1)+sqrt(2*x_F(2,1,2))*erfinv(2*Confidence_Level-1);
        % pr(x_F(2,1,1)<129)>CL
        c_F(exchange_2+4) = x_F(2,1,1)-129+sqrt(2*x_F(2,1,2))*erfinv(2*Confidence_Level-1);
    else
%           c_F(exchange_2+3) = 8-x_F(1,1,1);
%           c_F(exchange_2+4) = x_F(2,1,1)-129;
        % pr(x_F(1,1,1)>8)>CL
        c_F(exchange_2+3) = 8-x_F(1,1,1)+sqrt(2*x_F(1,1,2))*erfinv(2*Confidence_Level-1);
        % pr(x_F(2,1,1)<129)>CL
        c_F(exchange_2+4) = x_F(2,1,1)-129+sqrt(2*x_F(2,1,2))*erfinv(2*Confidence_Level-1);
    end
    
    % this means that the distance between the Follower and Leader should
    % be greater than safety_distance
    
    ceq_F = [];

end

function [c_F,ceq_F] = terminalconstraints_F(t,x_F, x_curr_L_measuredByF)
    global mpciterations dt
    c_F   = [];
    ceq_F = [];
end

function [A_F, b_F, Aeq_F, beq_F, lb_F, ub_F] = linearconstraints_F(t,x_F, u_F, x_curr_L_measuredByF)

    global dDelta_max_F v_max_F steering_ang_max_F
    A_F   = [];% x_F(7)<=steering_ang_max_F
    b_F   = [];
    Aeq_F = [];
    beq_F = [];
    lb_F  = [-1 -1]; % u_F(1)>=0,u_F(2)>=-dDelta_max_F, u_F(3)>=0 . throttle, rate of changes in streeing angle, brake
    ub_F  = [1  1]; % u_F(1)<=1,u_F(2)<=dDelta_max_F, u_F(3)<=dDelta_max_F
end

function x_F = SystemModel_F(t,x_F, u_F, dt)
    % this function gets the inputs and returns the state x
    % t: current time
    % x_F: the current state of the Follower
    % u_F: the control signal of the Follower
    % dt: time step

    
    global alpha_n_F T_m_F beta_F omega_m_F m_F theta_s g rho c_d_F A_F c_r_F a_F b_F mu_road braking_coefficient
    
    %{
    velocity_vec_angle=atan((a_F/b_F)*tan(x_F(7,1,1))); % (previous) angle between the velocity vector and central line of the vehicle
    velocity_F=cos(velocity_vec_angle)*sqrt((x_F(4,1,1))^2+(x_F(5,1,1))^2); % (previous) velocity of the Leader at the previous step
    F_F=alpha_n_F*u_F(1)*T_m_F*(1-beta_F*(1-alpha_n_F*velocity_F/omega_m_F)^2);% driving force of the Leader (u_L(1): position of throttle)
    %F_g_F= m_F*g*sin(theta_s);% gravity force working on the Leader
    F_r_F=m_F*g*c_r_F*velocity_F/(abs(velocity_F+eps)); % rolling friction force working on the Leader (sgn(v)=v/(abs(v+eps)))
    %F_a_F=0.5*rho*c_d_F*A_F*velocity_F^2; % aerodynamic drag working on the Leader
    F_b_F=u_F(3)*mu_road*m_F*g; % Braking force on the Leader
    velocity_F=velocity_F+dt*(1/m_F)*(F_F-F_b_F-F_r_F);% new velocity of the Leader
    
    x_F(7,1,1)=x_F(7,1,1)+dt*u_F(2);% steering angle of the Leader (new); (u_L(2): steering angular velocity)
    x_F(6,1,1)=(velocity_F/b_F)*(tan(x_F(7,1,1)))*a_F/b_F;% heading angular velocity of the Leader (new)
    x_F(3,1,1)=x_F(3,1,1)+dt*x_F(6,1,1); % heading angle of the Leader (new)
    x_F(5,1,1)=velocity_F*sin(x_F(3,1,1)+atan((tan(x_F(7,1,1)))*a_F/b_F))/cos(atan((tan(x_F(7,1,1)))*a_F/b_F)); % velocity at y-coord (new)  
    x_F(4,1,1)=velocity_F*cos(x_F(3,1,1)+atan((tan(x_F(7,1,1)))*a_F/b_F))/cos(atan((tan(x_F(7,1,1)))*a_F/b_F)); % velocity at x-coord (new)
    x_F(2,1,1)=x_F(2,1,1)+dt*x_F(5,1,1); % y-coordinate of the Leader (new)
    x_F(1,1,1)=x_F(1,1,1)+dt*x_F(4,1,1); % x-coordinate of the Leader (new)

    x_F(:,1,2)=x_F(:,1,2);% the variances of the variables will stay constant by time
    %}
    
    % simple dynamic model for vehicle ref: https://github.com/NikolasEnt/Model-Predictive-Control
  
    if (u_F(1)<0)
        u_F(1)= braking_coefficient*u_F(1);
    elseif ((u_F(2)<0))
        u_F(2)=braking_coefficient*u_F(2);
    end
    
    x_F(3,1,1)= x_F(3,1,1) + u_F(1)*dt; % velocity of the Leader at x-coordinate
    x_F(4,1,1)= x_F(4,1,1) + u_F(2)*dt; % velocity of the Leader at y-coordinate
    x_F(1,1,1)=x_F(1,1,1)+dt*x_F(3,1,1); % x-coordinate of the Leader (new)
    x_F(2,1,1)=x_F(2,1,1)+dt*x_F(4,1,1); % y-coordinate of the Leader (new)
 

end  


%%  SHARED FUNCTIONS

function min_dist=point_to_line(pt,xr)

% this function gets the coordinates of a point (pt) and finds its shortest
% distance from a path (in this case ref signal xr) (workd for 3-D as well).
% it finds the distance of the point pt from each line segment of given path xr and
% returns their minimum as the distance of the point from the path (min_dist).
% pt : coordinates of a point [x,y]
% xr : reference path including the vertices. format [x1 x2 , ... ; y1 y2 ...]

    distance_m=zeros(1,length(xr)-1);% zero matrix for each segment of line xr
    for i=1:length(xr)-1
        
        v1=xr(:,i); % ith vertex of line xr
        v2=xr(:,i+1); % (i+1)th vertex of line xr
        % distance of point pt from line segment composed of vertex i and
        % i+1
        if dot(v1-v2,pt-v2)*dot(v2-v1,pt-v1)>=0
            AA = [v1',1;v2',1;pt',1];
            dist = abs(det(AA))/norm(v1-v2);        
        else
            dist = min(norm(v1-pt), norm(v2-pt));
        end
        distance_m(1,i)=dist;% save the found distance in the matrix
    end
    min_dist=min(distance_m); % returns the min of the distance as min_dist

end

function Leader_plot=car_plot_L(x_L)

% this function takes the state of the car and plots it as the output
% the inputs are:
% x_L(1): x-coord of the Leader
% x_L(2): y-coord of the Leader
% x_L(3): heading angle of the Leader
% x_L(7): steering angle of the Leader

% the output is the a plot of car scheme


% position of the center of the mass is [0 0]


% rotate the car around its central point (center of mass) [0 0] as much as theta x_L(3)
% steer the wheels as much as delta x_L(7)
% and transform it to the desired point [x_L(1),x_L(2)]


%{
heading_angle=x_L(3);% heading angle x_L(3)
steering_angle=x_L(7);% steering angle
x_transfer=x_L(1);
y_transfer=x_L(2);

heading_rotation=[cos(heading_angle) -sin(heading_angle);sin(heading_angle) cos(heading_angle)];
steering_rotation=[cos(steering_angle) -sin(steering_angle);sin(steering_angle) cos(steering_angle)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
main_body1=[1.5500    0.0000; 1.5500   -0.8500; 1.3500   -0.9500; 1.3500   -0.2500
          -0.0500   -0.5500; -1.6500   -0.5500; -1.6500   -0.9000; -1.9500   -0.9500
         -1.9500    0.9500;-1.6500    0.9000;-1.6500    0.5500;-0.0500    0.5500
          1.3500    0.2500;1.3500    0.9500;1.5500    0.8500; 1.5500    0.0000];       
main_body1=main_body1*(heading_rotation^-1);
main_body1(:,1)=main_body1(:,1)+x_transfer;
main_body1(:,2)=main_body1(:,2)+y_transfer;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
main_body2 =[ 1.3500    0.2500; 1.3500   -0.2500];
main_body2=main_body2*(heading_rotation^-1);
main_body2(:,1)=main_body2(:,1)+x_transfer;
main_body2(:,2)=main_body2(:,2)+y_transfer;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
main_body3 =[ -1.6500    0.5500;  -1.6500   -0.5500];
main_body3=main_body3*(heading_rotation^-1);
main_body3(:,1)=main_body3(:,1)+x_transfer;
main_body3(:,2)=main_body3(:,2)+y_transfer;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tyresNE =[ 1.1500   -0.6500; 1.1500   -0.9500; 0.4500   -0.9500;0.4500   -0.6500
        1.1500   -0.6500; 1.1500   -0.9500; 0.4500   -0.9500; 0.4500   -0.6500];   
tyresNE_center=[(tyresNE(1,1)+tyresNE(3,1))/2 (tyresNE(1,2)+tyresNE(2,2))/2];
tyresNE(:,1)=tyresNE(:,1)-tyresNE_center(1);
tyresNE(:,2)=tyresNE(:,2)-tyresNE_center(2);
tyresNE=tyresNE*(steering_rotation^-1);
tyresNE(:,1)=tyresNE(:,1)+tyresNE_center(1);
tyresNE(:,2)=tyresNE(:,2)+tyresNE_center(2);
tyresNE=tyresNE*(heading_rotation^-1);
tyresNE(:,1)=tyresNE(:,1)+x_transfer;
tyresNE(:,2)=tyresNE(:,2)+y_transfer;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tyresNW =[ 1.1500    0.6500; 1.1500    0.9500; 0.4500    0.9500;0.4500    0.6500
     1.1500    0.6500; 1.1500    0.9500; 0.4500    0.9500;0.4500    0.6500];
tyresNW_center=[(tyresNW(1,1)+tyresNW(3,1))/2 (tyresNW(1,2)+tyresNW(2,2))/2];
tyresNW(:,1)=tyresNW(:,1)-tyresNW_center(1);
tyresNW(:,2)=tyresNW(:,2)-tyresNW_center(2);
tyresNW=tyresNW*(steering_rotation^-1);
tyresNW(:,1)=tyresNW(:,1)+tyresNW_center(1);
tyresNW(:,2)=tyresNW(:,2)+tyresNW_center(2);
tyresNW=tyresNW*(heading_rotation^-1);
tyresNW(:,1)=tyresNW(:,1)+x_transfer;
tyresNW(:,2)=tyresNW(:,2)+y_transfer;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tyresSE =[-0.8500   -0.6500;-0.8500   -0.9500;-1.5500   -0.9500;-1.5500   -0.6500
   -0.8500   -0.6500;-0.8500   -0.9500;-1.5500   -0.9500;-1.5500   -0.6500];     
tyresSE=tyresSE*(heading_rotation^-1);
tyresSE(:,1)=tyresSE(:,1)+x_transfer;
tyresSE(:,2)=tyresSE(:,2)+y_transfer; 
tyresSW =[-0.8500    0.6500;-0.8500    0.9500;-1.5500    0.9500;-1.5500    0.6500
     -0.8500    0.6500;-0.8500    0.9500; -1.5500    0.9500; -1.5500    0.6500];
tyresSW=tyresSW*(heading_rotation^-1);
tyresSW(:,1)=tyresSW(:,1)+x_transfer;
tyresSW(:,2)=tyresSW(:,2)+y_transfer;     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
axisN =[ 0.8100    0.7000; 0.8100   -0.7000; 0.7900   -0.7000; 0.7900    0.7000];
axisN=axisN*(heading_rotation^-1);
axisN(:,1)=axisN(:,1)+x_transfer;
axisN(:,2)=axisN(:,2)+y_transfer; 
axisS =[-1.1900    0.7000; -1.1900   -0.7000; -1.2100   -0.7000; -1.2100    0.7000];
axisS=axisS*(heading_rotation^-1);
axisS(:,1)=axisS(:,1)+x_transfer;
axisS(:,2)=axisS(:,2)+y_transfer;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
horizontal_radius=1.4/2; % horizontal radius
vertical_radius=0.9/2; % vertical radius
x0_center=-0.35; % x0_center,y0_center ellipse centre coordinates
y0_center=0;
t_step=-pi:0.1:pi;
x_cabin=x0_center+horizontal_radius*cos(t_step);
y_cabin=y0_center+vertical_radius*sin(t_step);
cabin=[x_cabin;y_cabin];
cabin=cabin'*(heading_rotation^-1);
cabin(:,1)=cabin(:,1)+x_transfer;
cabin(:,2)=cabin(:,2)+y_transfer;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Leader_plot{1,1}=patch(axisN(:,1),axisN(:,2),'k');
Leader_plot{2,1}=patch(axisS(:,1),axisS(:,2),'k');
Leader_plot{3,1}=patch(main_body1(:,1),main_body1(:,2),'r');
Leader_plot{4,1}=patch(main_body2(:,1),main_body2(:,2),'r');
Leader_plot{5,1}=patch(main_body3(:,1),main_body3(:,2),'r');
Leader_plot{6,1}=patch(tyresNE(:,1),tyresNE(:,2),'k');
Leader_plot{7,1}=patch(tyresNW(:,1),tyresNW(:,2),'k');
Leader_plot{8,1}=patch(tyresSE(:,1),tyresSE(:,2),'k');
Leader_plot{9,1}=patch(tyresSW(:,1),tyresSW(:,2),'k');
Leader_plot{10,1}=patch(cabin(:,1),cabin(:,2),'K','FaceAlpha',0.4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

ttt = linspace(0, 2*pi);
rrr = 1.25;
xxx = rrr*cos(ttt)+x_L(1);
yyy = rrr*sin(ttt)+x_L(2);

Leader_plot{1,1}=patch(xxx, yyy, 'r');

end

function Follower_plot=car_plot_F (x_F)

% this function takes the state of the car and plots it as the output
% the inputs are:
% x_F(1): x-coord of the Follower
% x_F(2): y-coord of the Follower
% x_F(3): heading angFe of the Follower
% x_F(7): steering angle of the Follower

% the output is the a plot of car scheme


% position of the center of the mass is [0 0]


% rotate the car around its central point (center of mass) [0 0] as much as theta x_F(3)
% steer the wheels as much as delta x_F(7)
% and transform it to the desired point [x_F(1),x_F(2)]


%{
heading_angle=x_F(3);% heading angle x_F(3)
steering_angle=x_F(7);% steering angle
x_transfer=x_F(1);
y_transfer=x_F(2);

heading_rotation=[cos(heading_angle) -sin(heading_angle);sin(heading_angle) cos(heading_angle)];
steering_rotation=[cos(steering_angle) -sin(steering_angle);sin(steering_angle) cos(steering_angle)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
main_body1=[1.5500    0.0000; 1.5500   -0.8500; 1.3500   -0.9500; 1.3500   -0.2500
          -0.0500   -0.5500; -1.6500   -0.5500; -1.6500   -0.9000; -1.9500   -0.9500
         -1.9500    0.9500;-1.6500    0.9000;-1.6500    0.5500;-0.0500    0.5500
          1.3500    0.2500;1.3500    0.9500;1.5500    0.8500; 1.5500    0.0000];       
main_body1=main_body1*(heading_rotation^-1);
main_body1(:,1)=main_body1(:,1)+x_transfer;
main_body1(:,2)=main_body1(:,2)+y_transfer;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
main_body2 =[ 1.3500    0.2500; 1.3500   -0.2500];
main_body2=main_body2*(heading_rotation^-1);
main_body2(:,1)=main_body2(:,1)+x_transfer;
main_body2(:,2)=main_body2(:,2)+y_transfer;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
main_body3 =[ -1.6500    0.5500;  -1.6500   -0.5500];
main_body3=main_body3*(heading_rotation^-1);
main_body3(:,1)=main_body3(:,1)+x_transfer;
main_body3(:,2)=main_body3(:,2)+y_transfer;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tyresNE =[ 1.1500   -0.6500; 1.1500   -0.9500; 0.4500   -0.9500;0.4500   -0.6500
        1.1500   -0.6500; 1.1500   -0.9500; 0.4500   -0.9500; 0.4500   -0.6500];   
tyresNE_center=[(tyresNE(1,1)+tyresNE(3,1))/2 (tyresNE(1,2)+tyresNE(2,2))/2];
tyresNE(:,1)=tyresNE(:,1)-tyresNE_center(1);
tyresNE(:,2)=tyresNE(:,2)-tyresNE_center(2);
tyresNE=tyresNE*(steering_rotation^-1);
tyresNE(:,1)=tyresNE(:,1)+tyresNE_center(1);
tyresNE(:,2)=tyresNE(:,2)+tyresNE_center(2);
tyresNE=tyresNE*(heading_rotation^-1);
tyresNE(:,1)=tyresNE(:,1)+x_transfer;
tyresNE(:,2)=tyresNE(:,2)+y_transfer;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tyresNW =[ 1.1500    0.6500; 1.1500    0.9500; 0.4500    0.9500;0.4500    0.6500
     1.1500    0.6500; 1.1500    0.9500; 0.4500    0.9500;0.4500    0.6500];
tyresNW_center=[(tyresNW(1,1)+tyresNW(3,1))/2 (tyresNW(1,2)+tyresNW(2,2))/2];
tyresNW(:,1)=tyresNW(:,1)-tyresNW_center(1);
tyresNW(:,2)=tyresNW(:,2)-tyresNW_center(2);
tyresNW=tyresNW*(steering_rotation^-1);
tyresNW(:,1)=tyresNW(:,1)+tyresNW_center(1);
tyresNW(:,2)=tyresNW(:,2)+tyresNW_center(2);
tyresNW=tyresNW*(heading_rotation^-1);
tyresNW(:,1)=tyresNW(:,1)+x_transfer;
tyresNW(:,2)=tyresNW(:,2)+y_transfer;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tyresSE =[-0.8500   -0.6500;-0.8500   -0.9500;-1.5500   -0.9500;-1.5500   -0.6500
   -0.8500   -0.6500;-0.8500   -0.9500;-1.5500   -0.9500;-1.5500   -0.6500];     
tyresSE=tyresSE*(heading_rotation^-1);
tyresSE(:,1)=tyresSE(:,1)+x_transfer;
tyresSE(:,2)=tyresSE(:,2)+y_transfer; 
tyresSW =[-0.8500    0.6500;-0.8500    0.9500;-1.5500    0.9500;-1.5500    0.6500
     -0.8500    0.6500;-0.8500    0.9500; -1.5500    0.9500; -1.5500    0.6500];
tyresSW=tyresSW*(heading_rotation^-1);
tyresSW(:,1)=tyresSW(:,1)+x_transfer;
tyresSW(:,2)=tyresSW(:,2)+y_transfer;     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
axisN =[ 0.8100    0.7000; 0.8100   -0.7000; 0.7900   -0.7000; 0.7900    0.7000];
axisN=axisN*(heading_rotation^-1);
axisN(:,1)=axisN(:,1)+x_transfer;
axisN(:,2)=axisN(:,2)+y_transfer; 
axisS =[-1.1900    0.7000; -1.1900   -0.7000; -1.2100   -0.7000; -1.2100    0.7000];
axisS=axisS*(heading_rotation^-1);
axisS(:,1)=axisS(:,1)+x_transfer;
axisS(:,2)=axisS(:,2)+y_transfer;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
horizontal_radius=1.4/2; % horizontal radius
vertical_radius=0.9/2; % vertical radius
x0_center=-0.35; % x0_center,y0_center ellipse centre coordinates
y0_center=0;
t_step=-pi:0.1:pi;
x_cabin=x0_center+horizontal_radius*cos(t_step);
y_cabin=y0_center+vertical_radius*sin(t_step);
cabin=[x_cabin;y_cabin];
cabin=cabin'*(heading_rotation^-1);
cabin(:,1)=cabin(:,1)+x_transfer;
cabin(:,2)=cabin(:,2)+y_transfer;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Follower_plot{1,1} =patch(axisN(:,1),axisN(:,2),'k');
Follower_plot{2,1} =patch(axisS(:,1),axisS(:,2),'k');
Follower_plot{3,1} =patch(main_body1(:,1),main_body1(:,2),'b');
Follower_plot{4,1} =patch(main_body2(:,1),main_body2(:,2),'b');
Follower_plot{5,1} =patch(main_body3(:,1),main_body3(:,2),'b');
Follower_plot{6,1} =patch(tyresNE(:,1),tyresNE(:,2),'k');
Follower_plot{7,1} =patch(tyresNW(:,1),tyresNW(:,2),'k');
Follower_plot{8,1} =patch(tyresSE(:,1),tyresSE(:,2),'k');
Follower_plot{9,1} =patch(tyresSW(:,1),tyresSW(:,2),'k');
Follower_plot{10,1}=patch(cabin(:,1),cabin(:,2),'K','FaceAlpha',0.4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

ttt = linspace(0, 2*pi);
rrr = 1.25;
xxx = rrr*cos(ttt)+x_F(1,1,1);
yyy = rrr*sin(ttt)+x_F(2,1,1);

Follower_plot{1,1}=patch(xxx, yyy, 'b');

end

function environment_plot_set=enviroment_plot

% this function draws the area around the vehicles
% including the street and the bulding
% we assume there are 2 lanes and they are seperated by a central line
 
street=[4    4   7    137  137  14   12  12
        0   130  133  133  125  125  123  0];

central_line=[8  8        10.5    137
              0  126.5    129     129];
   
area_1 = [12  12    14    137  137
           0  123   125   125   0];

area_2 = [0  0    137  137   7      4     4
          0  137  137  133   133    130   0];

environment_plot_set{1}=patch(area_1(1,:),area_1(2,:),[0.8 0.4 0],'FaceAlpha',0.4);
environment_plot_set{2}=patch(area_2(1,:),area_2(2,:),[0.8 0.4 0],'FaceAlpha',0.4);
environment_plot_set{3}=patch(street(1,:),street(2,:),[0.1 0.1 0.1],'FaceAlpha',0.8);
environment_plot_set{4}=plot(central_line(1,:), central_line(2,:),'--','color',[1 1 1]);

end

function draw_diagram(u_L,x_L,overall_velocity_L,objective_L,u_F,x_F,overall_velocity_F,objective_F)
% this function gets u_L,x_L,u_F,x_F and draws some graphs to compare the
% results. 
global mpciterations dt safety_distance v_max_L

%--------------------------------------------------------------------------
% safety distance and current distance between Leader and Follower
subplot(3,2,2)
hold on
grid minor

distance_F_L=zeros(mpciterations,1); % this matrix shows the distance between two vehicles at different time steps
for i=1:mpciterations
    distance_F_L(i)=sqrt((x_F(1,i,1)-x_L(1,i))^2+(x_F(2,i,1)-x_L(2,i))^2);
end
plot(1:mpciterations,safety_distance*ones(mpciterations,1),'g')
plot(1:mpciterations,distance_F_L,'k')
title('Distance Between Two Vehicles')
%--------------------------------------------------------------------------
% overall speed of Leader and Follower
subplot(3,2,4)
hold on
grid minor

plot(1:mpciterations+1,overall_velocity_L,'r')
plot(1:mpciterations+1,overall_velocity_F,'b')
title('Overall Velocity')
%--------------------------------------------------------------------------
% fuel consumption of Follower (cumulative)
subplot(3,2,5)
hold on
grid minor

%{
%acceleration of Follower and Leader at x-coord
plot(1:mpciterations, u_L(1,1:mpciterations),'r')
plot(1:mpciterations, u_F(1,1:mpciterations),'b')
title('Acceleration at X-coordinate')
%}
%{
% objective function of Leader
plot(1:mpciterations,objective_L(1:mpciterations,1),'r')
axis([0 mpciterations 0 v_max_L*1.2])
title('Leader`s Objective Function')
%}
fuelConsum_F_result=zeros(mpciterations,1); % this matrix shows the amount of fuel consumption in different time steps
cumulative_fuelConsum_F=zeros(mpciterations,1);
for i=1:mpciterations
    speed_F=3.28*overall_velocity_F(i);% speed_F(ft/s) and overall_velocity_F(m/s) and
    % 1 m/s is equal to 3.28 ft/s
    acceleration_F=3.28*sqrt((u_F(1,i))^2+(u_F(2,i))^2);
    % 1 m/s/s is equal to 3.28 ft/s/s
    fuelConsum_F_result(i)=dt*(exp((10^-5)*((20+0.3*speed_F/2+0.04*(speed_F^2)/2))*(1+acceleration_F+acceleration_F^2))-1); 
    cumulative_fuelConsum_F(i)=sum(fuelConsum_F_result(1:i));% cumulative fuel consumption by now
end
plot(1:mpciterations, cumulative_fuelConsum_F,'b')
title('Follower`s Fuel consumption liter/sec')
%--------------------------------------------------------------------------
% objective function of Follower
subplot(3,2,6)
hold on
grid minor
%{
%acceleration of Follower and Leader at y-coord
plot(1:mpciterations, u_L(2,1:mpciterations),'r')
plot(1:mpciterations, u_F(2,1:mpciterations),'b')
title('Acceleration at Y-coordinate')
%}

plot(1:mpciterations,objective_F(1:mpciterations,1),'b')
title('Follower`s Objective Function')
%--------------------------------------------------------------------------

end












