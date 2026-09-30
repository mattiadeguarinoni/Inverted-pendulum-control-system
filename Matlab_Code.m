
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%                                                         %%%%%%%%%
%%%%%%%%%       ANALISI E SIMULAZIONE DI SISTEMI AEROSPAZIALI     %%%%%%%%%
%%%%%%%%%                       A.A. 2025-2026                    %%%%%%%%%
%%%%%%%%%                                                         %%%%%%%%%
%%%%%%%%%                          GRUPPO 22                      %%%%%%%%%
%%%%%%%%%                        LABORATORIO 01                   %%%%%%%%%
%%%%%%%%%                                                         %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



clearvars
Pend = Pend_parameters();
Pend.T1 = 1;
Pend.T2 = 3;
% Task 2

% Task2.1
% The system of nonlinear equations of motion, implemented in the function
% Pend_f, was numerically integrated using MATLAB built-in ODE solvers, to
% which all the required parameters were supplied via the function Pend.
% The disturbance input, managed by the function Pend_input, was set to
% zero by imposing Pend.d_max = 0, since in Task 2.1 only the free
% response of the system to initial conditions is investigated, which are
% stored in the vector x0. The remaining input parameters include the
% damping coefficients, assumed to be non-zero, and the controller gains
% K_p and K_d, both set to zero as they are relevant only for subsequent
% analyses.
%
% The solver object ode.obj was configured by selecting the ode89 method
% with default tolerances. The obtained results, together with the step
% size history adopted by the solver, were plotted. The procedure was
% subsequently repeated employing the ode23 and ode45 solvers, in order to
% compare the respective integration step diagrams. The overall results
% were finally collected and illustrated in the figures reported below.

t_0 = 0;
t_f = 100;
x0 = [0;0;deg2rad(1);0];
Pend.d_max = 0;
ODE_obj = ode;
ODE_obj.ODEFcn = @(t,x) Pend_f(t,x,@Pend_input,Pend,Pend.c,Pend.b,0,0);
ODE_obj.InitialValue = x0;
ODE_obj.Solver = 'ode89';
% The tolerances were used only for the ode45 solver to achieve the 
% exptected solution
% ODE_obj.AbsoluteTolerance = 1e-12;
% ODE_obj.RelativeTolerance = 1e-6;
ODEResults_obj = solve(ODE_obj,t_0,t_f);
t_x0 = ODEResults_obj.Time'; % integration time steps non-linear solution C.I.
x_x0 = ODEResults_obj.Solution'; % integration non linear solution C.I.
figure('Name','Non-linear model response to initial conditions');
tiledlayout(2, 1);
nexttile;
plot(t_x0, x_x0(:,1), 'b', 'LineWidth', 1.5);
title('Cart position','FontSize',10);
xlabel('Time [s]','FontSize', 10);
ylabel('x [m]','FontSize',10);
set(gca, 'FontSize', 10);
grid on;
nexttile;
plot(t_x0, rad2deg(x_x0(:,3)), 'r', 'LineWidth', 1.5);
title('Pendulum angle','FontSize',10);
xlabel('Time [s]','FontSize',10);
ylabel('\theta [deg]','FontSize',10);
set(gca, 'FontSize', 10);
ylim([0 330]);
grid on;
hvec = zeros(length(t_x0),1);
for n = 2:length(t_x0)
   hvec(n) = t_x0(n)-t_x0(n-1);
end
figure('Name','adaptive step sizes response to initial conditions')
plot(1:length(t_x0),hvec);
xlabel('n','FontSize',22);
ylabel('h','FontSize',22);
xlim([1 length(t_x0)]);
set(gca, 'FontSize', 15);
grid on

% Comparison between all solvers
% In order to obtain a single figure displaying the simulation results
% obtained with different solvers, a code was developed to store the
% respective outputs into a vector, following the same logic
% described in the previous section. In addition to the MATLAB built-in
% ODE solvers, the Forward Euler and Backward Euler methods were also
% implemented and included in the comparison. The resulting solutions were
% finally collected and illustrated in a single plot reported below.

solvers = {'ode23', 'ode45', 'ode89'};
colors  = {'g',     'b',     'r'};
results = cell(length(solvers), 1);

for i = 1:length(solvers)
    ODE_obj = ode;
    ODE_obj.ODEFcn = @(t,x) Pend_f(t,x,@Pend_input,Pend,Pend.c,Pend.b,0,0);
    ODE_obj.InitialValue = x0;
    ODE_obj.Solver = solvers{i};
    results{i} = solve(ODE_obj, t_0, t_f);
end

h_e = 0.01;% Integration step size fot the Forward and Backward euler methods
% Forward Euler method
t_span_euler = t_0:h_e:t_f;
ode_fun = @(t, x, input, Pend) Pend_f(t, x, input, Pend, Pend.c, Pend.b, 0, 0);
[t_ef, x_ef] = forward_euler(ode_fun, t_span_euler, x0, Pend);

% Backward euler method 
ode_fun_b = @(t, x) Pend_f(t, x, @Pend_input, Pend, Pend.c, Pend.b, 0, 0);
[t_eb, x_eb] = ode_Euler_b(ode_fun_b, [t_0, t_f], x0, h_e);
x_eb = x_eb'; 

figure('Name','Comparison between solutions of the different solvers');
tiledlayout(2, 1);
nexttile;
hold on;
for i = 1:length(solvers)
    t = results{i}.Time';
    x = results{i}.Solution';
    plot(t, x(:,1), colors{i}, 'LineWidth', 1.5);
end
plot(t_ef, x_ef(:,1), 'm',  'LineWidth', 1.5);
plot(t_eb, x_eb(:,1), 'k',  'LineWidth', 1.5);
title('Cart position');
xlabel('Time [s]');
ylabel('x [m]');
legend([solvers, {'Forward Euler', 'Backward Euler'}], 'Location', 'best');
grid on;

nexttile;
hold on;
for i = 1:length(solvers)
    t_p = results{i}.Time';
    x_p = results{i}.Solution';
    plot(t_p, rad2deg(x_p(:,3)), colors{i}, 'LineWidth', 1.5);
end

plot(t_ef, rad2deg(x_ef(:,3)), 'm', 'LineWidth', 1.5);
plot(t_eb, rad2deg(x_eb(:,3)), 'k', 'LineWidth', 1.5);
title('Pendulum angle');
xlabel('Time [s]');
ylabel('\theta [deg]');
legend([solvers, {'Forward Euler', 'Backward Euler'}], 'Location', 'best');
grid on;

% Backward Euler method computational time
% The code solve the equations of motion and calculate the computational 
% time needed to evaluate an avarage of it
% N_iterations = 5;
% Time_eb = zeros(N_iterations,1);
% for k = 1:N_iterations
%     tic;
%     h = 0.1;
%     tspan_eb = [t_0,t_f];
%     fun = @(t,x) Pend_f(t,x,@Pend_input,Pend,Pend.c,Pend.b,0,0);
%     [t_eb,x_eb] = ode_Euler_b(fun,tspan_eb,x0,h);
%     Time_eb(k) = toc;
% end
% mean_time = mean(Time_eb);


% Task 2.2
% The approach adopted in this section follows the same methodology 
% outlined in Task 2.1.
% In this case, however, the disturbance amplitude is set as
% Pend.d_max = 1, and the initial conditions are reset to x0d = [0;0;0;0],
% denoted with a different notation to distinguish them from the previous
% ones. The observation time is additionally limited to 50 s. As in the
% previous section, the integration step size history was computed and the
% obtained results were illustrated in the figures reported below.


Pend.d_max = 1;
d0=0.3;
x0d = [0;0;0;0];
t_0 = 0;
t_fd = 50;
ODE_obj = ode;
ODE_obj.ODEFcn = @(t,x) Pend_f(t,x,@Pend_input,Pend,Pend.c,Pend.b,0,0);
ODE_obj.InitialValue = x0d;
ODE_obj.Solver = 'ode45';
ODEResults_obj = solve(ODE_obj,t_0,t_fd);
t_d = ODEResults_obj.Time'; % integration non-linear time steps D 
x_d = ODEResults_obj.Solution'; % integration non-linear solution D
d = zeros(length(t_d),1); 
for i = 1: length(t_d)
   d(i,:) = Pend_input(t_d(i),Pend);
end
hvec_d = zeros(length(t_d),1);
for n = 2:length(t_d)
   hvec_d(n) = t_d(n)-t_d(n-1);
end

figure('Name','Non linear model response to disturbance');
tiledlayout(2, 1);
nexttile;
yyaxis left
plot(t_d, x_d(:,1), 'b', 'LineWidth', 1.5);
title('Cart position','FontSize',10);
xlabel('Time [s]','FontSize',10);
ylabel('x [m]','FontSize',10);
yyaxis right
plot(t_d,d0*d,'--','LineWidth',1.5)
ylabel('Disturbance','FontSize',10)
grid on;
set(gca, 'FontSize', 10);
nexttile;
yyaxis left;
plot(t_d, rad2deg(x_d(:,3)), 'r', 'LineWidth', 1.5);
title('Pendulum angle','FontSize',10);
xlabel('Time [s]','FontSize',10);
ylabel('\theta [deg]','FontSize',10);
yyaxis right
plot(t_d,d0*d,'--','LineWidth',1.5)
ylabel('Disturbance','FontSize',10)
set(gca, 'FontSize', 10);
grid on
figure('Name','Adaptive step sizes response to disturbance')
plot(1:length(d),hvec_d);
xlabel('n','FontSize',22);
ylabel('h','FontSize',22);
xlim([1 length(t_d)]);
set(gca, 'FontSize', 15);
grid on

% The code is created to evaluate an average of computational time cost for
% the backward euelr method with disturbance
% Backwards euler method  with disturbance
% N_iterations = 5;
% Time_eb_d = zeros(N_iterations,1);
% for k = 1:N_iterations
%     tic;
%     h = 0.01;
%     tspan_eb = [t_0,t_f];
%     fun = @(t,x) Pend_f(t,x,@Pend_input,Pend,Pend.c,Pend.b,0,0);
%     [t_eb_d,x_eb_d] = ode_Euler_b(fun,tspan_eb,x0d,h);
%     Time_eb_d(k) = toc;
% end
% mean_time_d = mean(Time_eb_d);
% plot(t_eb_d,x_eb_d(1,:))




% The code is created with the same logic as the code in the response to
% initial conditions section to evaluate and plot the response of the
% system to the disturbance. To differentiate the "_d" is used.

solvers = {'ode23', 'ode45', 'ode89'};
colors  = {'g',     'b',     'r'};
results_d = cell(length(solvers), 1);
Pend.d_max = 1;
for i = 1:length(solvers)
    ODE_obj = ode;
    ODE_obj.ODEFcn = @(t,x) Pend_f(t,x,@Pend_input,Pend,Pend.c,Pend.b,0,0);
    ODE_obj.InitialValue = x0d;
    ODE_obj.Solver = solvers{i};
    results_d{i} = solve(ODE_obj, t_0, t_fd);
end

h_e = 0.01; % Integration step size fot the Forward and Backward euler methods
% Backward Euler method
ode_fun_b = @(t, x) Pend_f(t, x, @Pend_input, Pend, Pend.c, Pend.b, 0, 0);
[t_eb_d, x_eb_d] = ode_Euler_b(ode_fun_b, [t_0, t_f], x0, h_e);
x_eb_d = x_eb_d';

% Plot
figure('Name','Comparison between different solvers response to disturbance');
tiledlayout(2, 1);

nexttile;
hold on;
for i = 1:length(solvers)
    t = results_d{i}.Time';
    x = results_d{i}.Solution';
    plot(t, x(:,1), colors{i}, 'LineWidth', 1.5);
end
plot(t_eb_d, x_eb_d(:,1), 'k',  'LineWidth', 1);
title('Cart position','FontSize',14);
xlabel('Time [s]','FontSize',14);
ylabel('x [m]','FontSize',14);
legend([solvers, {'Backward Euler'}], 'Location', 'best');
grid on;

nexttile;
hold on;
for i = 1:length(solvers)
    t = results_d{i}.Time';
    x = results_d{i}.Solution';
    plot(t, rad2deg(x(:,3)), colors{i}, 'LineWidth', 1);
end

plot(t_eb_d, rad2deg(x_eb_d(:,3)), 'k', 'LineWidth', 1);
title('Pendulum angle','FontSize',14);
xlabel('Time [s]','FontSize',14);
ylabel('\theta [deg]','FontSize',14);
legend([solvers, {'Backward Euler'}], 'Location', 'best');
grid on;





% Task 3 
% A Simulink model was developed incorporating a MATLAB Function block, in
% which the Pend_f function was embedded. The disturbance input was
% generated by means of Step blocks, while the numerical integration was
% performed using an Integrator block. The resulting system response was
% finally visualized through a Scope block.
open_system('Model_22_01_1.slx')


% Task 4
% Task 4.2
% The linearized equations of motion, implemented in the function Pend_z,
% were numerically solved by supplying the function with the system
% parameters Pend and the disturbance input, the latter set to zero since
% the free response to initial conditions is investigated in this section.
% The initial conditions vector x0, defined in Task 2.1, was therefore
% employed. The results were subsequently plotted, with particular
% attention to the axis limits, in order to clearly highlight the system
% transient behaviour, which is predominantly concentrated in the early
% stage of the dynamics. A second figure was additionally produced to
% compare the response of the linearized system with that of the nonlinear
% one, so as to emphasize the discrepancies between the two models.

Pend.d_max = 0;
ODE_obj.ODEFcn = @(t,x) Pend_z(t,x,@Pend_input,Pend,Pend.c,Pend.b);
ODE_obj.InitialValue = x0;
ODE_obj.Solver = 'ode45';
ODEResults_obj = solve(ODE_obj,t_0,t_f);
t_z = ODEResults_obj.Time'; 
z = ODEResults_obj.Solution';
figure('Name','Linear-system response to intial conditions');
tiledlayout(2, 1);
nexttile;
plot(t_z, z(:,1), 'b', 'LineWidth', 1.5);
title('Cart position','FontSize',10);
xlabel('Time [s]','FontSize',10);
ylabel('x [m]','FontSize',10);
ylim([-2 4])
xlim([0 1.8])
set(gca, 'FontSize', 10);
grid on;
nexttile;
plot(t_z, rad2deg(z(:,3)), 'r', 'LineWidth', 1.5);
title('Pendulum angle','FontSize',10);
xlabel('Time [s]','FontSize',10);
ylabel('\theta [deg]','FontSize',10);
ylim([0 350])
set(gca, 'FontSize', 10);
grid on;

figure("name", "x comparison between linear and non-linear system")
plot(t_z, z(:,1), 'b', 'LineWidth', 1.5);
title('Cart position','FontSize',15);
xlabel('Time [s]','FontSize',15);
ylabel('x [m]','FontSize',15);
set(gca, 'FontSize', 15);
hold on;
plot(t_x0, x_x0(:,1), 'r', 'LineWidth', 1.5);
title('Cart position','FontSize',15);
xlabel('Time [s]','FontSize',15);
ylabel('x [m]','FontSize',15);
ylim([-0.5 1])
xlim([0 1.8])
set(gca, 'FontSize', 15);
grid on;
legend('Linear model','Non linear model')

figure("Name", "theta comparison between linear and non-linear system")
plot(t_z, rad2deg(z(:,3)), 'r', 'LineWidth', 1.5);
title('Pendulum angle','FontSize',15);
xlabel('Time [s]','FontSize',15);
ylabel('\theta [deg]','FontSize',15);
ylim([0 100])
set(gca, 'FontSize', 15);
grid on;
hold on
plot(t_x0, rad2deg(x_x0(:,3)), 'b', 'LineWidth', 1.5);
legend('Linear model','Non linear model')
%%
% Comparison between linear model and state space model
% A comparison was carried out between two methods for solving the
% linearized equations of motion. While the previous section employed a
% numerical ODE solver, in this case the MATLAB built-in command initial
% was adopted, which allows the free response of a linear system to
% prescribed initial conditions to be computed directly. The command
% returns both the solution vector and the corresponding time vector,
% which were then used to produce the comparison figure reported below.

[A,B,C,D] = Pend_sys(Pend,Pend.c,Pend.b);
Pend_sys_ss = ss(A,B,C,D);
[x_ss,t_ss] = initial(Pend_sys_ss,x0,50);

figure('Name','x/theta comparison ode-intial linear model');
tiledlayout(2, 1);
nexttile;
plot(t_ss, x_ss(:,1), 'b', 'LineWidth', 1.5);
hold on
plot(t_z, z(:,1), 'r', 'LineWidth', 1.5);
title('Cart position','FontSize',10);
xlabel('Time [s]','FontSize',10);
ylabel('x [m]','FontSize',10);
ylim([-1 5]);
set(gca, 'FontSize', 10);
grid on;
legend('State space','Ode')
nexttile;
plot(t_ss, rad2deg(x_ss(:,2)), 'b', 'LineWidth', 1.5);
hold on
plot(t_z, rad2deg(z(:,3)), 'r', 'LineWidth', 1.5);
title('Pendulum angle','FontSize',10);
xlabel('Time [s]','FontSize',10);
ylabel('\theta [deg]','FontSize',10);
legend('State space','Ode')
ylim([0 100]);
grid on;

%Task 4.3-4.4
% In order to verify the analytical derivation of the transfer functions,
% the MATLAB built-in command "tf" was applied to the frictionless linearized
% system, as required by the task. The command computes the transfer
% functions and assembles them into a matrix denoted G_tf. The state-space
% matrices labeled with the suffix no_fr refer to the linearized,
% frictionless system representation.
[A_no_fr,B_no_fr,C_no_fr,D_no_fr] = Pend_sys(Pend,0,0);
Pend_sys_ss = ss(A_no_fr,B_no_fr,C_no_fr,D_no_fr);
G_tf = tf(Pend_sys_ss);
Gthetai = G_tf(2,1);
Gthetad = G_tf(2,2);
Gxi = G_tf(1,1);
Gxd = G_tf(1,2);

% Task 4.5
% The poles and zeros of the transfer functions were plotted in the
% complex plane.
figure('Name','Transfer Functions');
tiledlayout(2, 2);
nexttile
pzplot(Gthetai);
title("G_\theta_i(s)","FontSize",10)
grid on
nexttile;
pzplot(Gthetad);
title("G_\theta_d(s)","FontSize",10)
grid on
nexttile;
pzplot(Gxi)
title("G_x_i(s)","FontSize",10)
grid on
nexttile;
pzplot(Gxd);
title("G_x_d(s)","FontSize",10)
grid on

%Task 5
% Task 5.4
kt = Pend.kt;
r = Pend.r;
d0 = Pend.d0;
g = Pend.g;
m = Pend.m;
M = Pend.M;
l = Pend.l;
am = kt/r;
I1 = m*l/2;
I2 = m*l^2/3;
a0 = d0*l;
a1 = d0*l^2/2;

% The values of the controller gains Kp and Kd were computed so as to
% satisfy the prescribed conditions on the natural frequency omega and the
% damping ratio xi, based on analytically derived relations. The transfer
% function of the complete controlled system, denoted G, was subsequently
% constructed by multiplying the individual transfer functions together.
% To include the controller transfer function in the product, the tf
% command was additionally applied to the Laplace variable s, making it
% available as a symbolic transfer function object. At the end of the code,
% the variable s was cleared in order to avoid conflicts in subsequent
% sections of the script. It is noted that throughout the code the variable
% w is used to denote the natural frequency omega, consistent with the
% notation adopted in the report.
w = 15;
xi = 0.7;
max = exp(-xi*pi/sqrt(1-xi^2));
kp_pd = (w^2*((M+m)*I2-I1^2)+(M+m)*I1*g)/(I1*am);
kd_pd = (2*xi*w*((M+m)*I2-I1^2))/(I1*am);
s = tf('s');
G = minreal(Gthetad/(1+(kp_pd+s*kd_pd)*Gthetai));
% Minimal realization: Floating-point arithmetic during algebraic tf manipulation 
% can create near-canceling pole-zero pairs. 'minreal' strips out these unobservable 
% and uncontrollable modes (hidden modes). If left un-canceled, these numerical 
% artifacts would artificially inflate the order of the controller and cause 
% numerical instability in step() and bode() evaluations.

figure('Name','system transfer function response to step');
stepinfo(G)
step(G)
grid on
figure('Name','Bode phase and mangnitude diagrams');
bode(G)
grid on
clear s

% Task 5.6
% The response of the nonlinear frictionless system to the disturbance
% input in the presence of the controller was simulated by means of the
% ode45 solver. The relevant variables are defined with the suffix _pd to
% refer to the controlled system configuration. The control variable which
% is the current provided by a motor, denoted i, is defined as 
% specified in the task. The results, together with the time history of the 
% control variable i, were finally illustrated in the figures reported below.
Pend.d_max = 1;
ODE_obj = ode;
ODE_obj.ODEFcn = @(t,x) Pend_f(t,x,@Pend_input,Pend,0,0,kp_pd,kd_pd);
ODE_obj.InitialValue = x0d;
ODE_obj.Solver = 'ode45'; 
ODEResults_obj = solve(ODE_obj,t_0,20);
t_pd = ODEResults_obj.Time';
x_pd = ODEResults_obj.Solution';
i_pd = -kp_pd*x_pd(:,3)-kd_pd*x_pd(:,4);
d_pd = zeros(length(t_pd),1);
for i = 1: length(t_pd)
   d_pd(i,:) = Pend_input(t_pd(i),Pend);
end

figure('Name','Pd control over theta in responce to disturbance frictionless');
tiledlayout(3, 1);
nexttile;
yyaxis left
plot(t_pd, x_pd(:,1), 'b', 'LineWidth', 1.5);
title('Cart position');
xlabel('Time [s]');
ylabel('x [m]');
ylim([-40 5]);
yyaxis right
plot(t_pd,d0*d_pd,'--');
ylabel('Disturbance')
grid on;
nexttile;
yyaxis left
plot(t_pd, rad2deg(x_pd(:,3)), 'r', 'LineWidth', 1.5);
title('Pendulum angle');
xlabel('Time [s]');
ylabel('\theta [deg]');
ylim([-0.2 0.7])
yyaxis right
plot(t_pd,d0*d_pd,'--');
ylabel('Disturbance')
grid on;
nexttile;
yyaxis left
plot(t_pd,i_pd,'LineWidth',1.5);
title('Current');
xlabel('Time [s]');
ylabel('i [A]');
yyaxis right
plot(t_pd,d0*d_pd,'--');
ylabel('Disturbance')
grid on;

% Task 5.7
% The effect of the controller on the nonlinear system with friction was
% assessed and the obtained results are illustrated in the figures reported
% below.
Pend.d_max = 1;
ODE_obj = ode;
ODE_obj.ODEFcn = @(t,x) Pend_f(t,x,@Pend_input,Pend,Pend.c,Pend.b,kp_pd,kd_pd);
ODE_obj.InitialValue = x0d;
ODE_obj.Solver = 'ode45'; 
ODEResults_obj = solve(ODE_obj,t_0,60);
t_pd_fr = ODEResults_obj.Time'; % integration time steps non-linear model with friction
x_pd_fr = ODEResults_obj.Solution'; % non-linear integration solutions 
i_pd_fr = -kp_pd*x_pd_fr(:,3)-kd_pd*x_pd_fr(:,4);
d_pd_fr = zeros(length(t_pd_fr),1); 
for i = 1: length(t_pd_fr)
   d_pd_fr(i,:) = Pend_input(t_pd_fr(i),Pend);
end

figure('Name','Pd control over theta in responce to disturbance with friction');
tiledlayout(3, 1);
nexttile;
yyaxis left
plot(t_pd_fr, x_pd_fr(:,1), 'b', 'LineWidth', 1.5);
title('Cart position');
xlabel('Time [s]');
ylabel('x [m]');
ylim([-200 5])
yyaxis right
plot(t_pd_fr,d0*d_pd_fr,'--')
ylabel('Disturbance')
grid on;
nexttile;
yyaxis left
plot(t_pd_fr, rad2deg(x_pd_fr(:,3)), 'r', 'LineWidth', 1.5);
title('Pendulum angle');
xlabel('Time [s]');
ylabel('\theta [deg]');
ylim([-0.2 0.7])
yyaxis right
plot(t_pd_fr,d0*d_pd_fr,'--')
ylabel('Disturbance')
grid on;
nexttile;
yyaxis left
plot(t_pd_fr,i_pd_fr,'LineWidth',1.5);
title('Current');
xlabel('Time [s]');
ylabel('i [A]');
ylim([-0.4 0.1])
yyaxis right
plot(t_pd_fr,d0*d_pd_fr,'--')
ylabel('Disturbance')
grid on;

% Task 5.8-5.10
% The PID controller was modeled and the corresponding transfer function
% was derived and implemented.
p3 = 20; % third fast real pole 
kd_pid = ((2*xi*w+p3)*((M+m)*I2-I1^2))/(I1*am);
ki_pid = (w^2*p3*((M+m)*I2-I1^2))/(I1*am);
kp_pid = ((w^2+2*xi*w*p3)*((M+m)*I2-I1^2)+(M+m)*I1*g)/(I1*am);
s = tf('s');
G = minreal(Gthetad/(1+(kp_pid+s*kd_pid+1/s*ki_pid)*Gthetai));


figure('Name','PID complete model Step response');
stepinfo(G)
step(G)
grid on
clear s

% Task 5.11
% The complete system with friction, regulated by the PID controller, was
% simulated and the obtained results are illustrated in the figures
% reported below. 
Pend.d_max = 1;
ODE_obj = ode;
ODE_obj.ODEFcn = @(t,x) Pend_f_5(t,x,@Pend_input,Pend,Pend.c,Pend.b,kp_pid,kd_pid,ki_pid);
ODE_obj.InitialValue = [0;0;0;0;0];
ODE_obj.Solver = 'ode45'; 
ODEResults_obj = solve(ODE_obj,t_0,60);
t_pid_fr = ODEResults_obj.Time'; % Integration time steps PID with friction
x_pid_fr = ODEResults_obj.Solution'; % integration slution PID with friction
i_pid_fr = -kp_pid*x_pid_fr(:,3)-kd_pid*x_pid_fr(:,4)-ki_pid*x_pid_fr(:,5);
d_pid_fr = zeros(length(t_pid_fr),1); 
for i = 1: length(t_pid_fr)
   d_pid_fr(i,:) = Pend_input(t_pid_fr(i),Pend);
end

figure('Name','PID regulated complete model response to disturbance');
tiledlayout(3, 1);
nexttile;
yyaxis left
plot(t_pid_fr, x_pid_fr(:,1), 'b', 'LineWidth', 1.5);
ylim([-110 5])
title('Cart position');
xlabel('Time [s]');
ylabel('x [m]');
hold on
yyaxis right
plot(t_pid_fr,d0*d_pid_fr,'--');
ylabel('Disturbance')
grid on;
nexttile;
yyaxis left
plot(t_pid_fr, rad2deg(x_pid_fr(:,3)), 'r', 'LineWidth', 1.5);
title('Pendulum angle');
xlabel('Time [s]');
ylabel('\theta [deg]');
ylim([-0.2 0.2])
yyaxis right
plot(t_pid_fr,d0*d_pid_fr,'--');
ylabel('Disturbance')
grid on;
nexttile;
yyaxis left
plot(t_pid_fr,i_pid_fr,'LineWidth',1.5);
title('Current');
xlabel('Time [s]');
ylabel('i [A]');
yyaxis right
plot(t_pid_fr,d0*d_pid_fr,'--');
ylabel('Disturbance')
grid on;

% Task 6.1
% The controllability of the system was verified by means of the MATLAB
% built-in command ctrb.

Pend.d_max = 1;
[A,B,C,D] = Pend_sys(Pend,0,0);
B_i = B(:,1);
Co = ctrb(A,B_i);
n = size(A,1);
isControllable = (rank(Co)==n); 

% Task 6.2
% The gain matrix K was computed by means of the pole-placement technique.
% Pole placement trade-off: While placing poles further into the Left Half-Plane (LHP) 
% guarantees faster asymptotic decay mathematically, it requires exponentially 
% larger feedback gains (matrix K). In a real physical setup, high K values 
% lead to control signal saturation (the motor cannot supply infinite current/torque). 
% These pole locations were chosen to balance a fast settling time with the physical 
% limitations of the motor's current draw. 
w_d = w*sqrt(1-xi^2);
Re_P1 = xi*w;
Im_P1 = w_d;
Pc_1 = -Re_P1 + 1i*Im_P1;
Pc_2 = -Re_P1 - 1i*Im_P1;
Pc_3 = -7;
Pc_4 = -9;
Pc = [Pc_1 Pc_2 Pc_3 Pc_4];
K = place(A,B_i,Pc);
Ac = A-B_i*K;
Pend_sys_x0 = ss(Ac,B,C,D);

% Closed-Loop simulation to intial conditions with pole-placement
% technique. The results are denoted as _x0 because are the response to the
% intial conditions and _pp beacuse they derive from the system regulated
% by a controller obtained by the pole-placement technique
[~,t_x0_pp,x_x0_pp] = initial(Pend_sys_x0,x0,20);
i_x0_pp= -( K(1)*x_x0_pp(:,1) + K(2)*x_x0_pp(:,2) + K(3)*x_x0_pp(:,3) + ...
    K(4)*x_x0_pp(:,4) );

figure('Name','Closed-Loop simulation to intial conditions with pole-placement technique');
tiledlayout(3, 1);
nexttile;
plot(t_x0_pp,x_x0_pp(:,1),'r','LineWidth',1.5);
title('Cart position')
xlabel('Time [s]');
ylabel('x [m]');
ylim([-0.020,+0.002])
grid on
nexttile;
plot(t_x0_pp,rad2deg(x_x0_pp(:,3)),'b','LineWidth',1.5);
title('Pendulum angle')
xlabel('Time [s]');
ylabel('\theta [deg]');
ylim([-1.3,+1])
grid on
nexttile;
plot(t_x0_pp,i_x0_pp,'LineWidth',1.5);
title('Current');
xlabel('Time [s]');
ylim([-2.4,+1])
ylabel('i [A]');
grid on
t_sim = 0:0.01:20;
U_input = zeros(length(t_sim), 2);
for k = 1:length(t_sim)
   t_attuale = t_sim(k);
   U_input(k, 2) = Pend_input(t_attuale, Pend);
end

% Closed-loop simulation to disturbance with pole-placement technique.
% the results are denoted as _d because they represent the response to the
% disturbance and _pp because they derive from the system regulated by a
% controller obtained by the pole-placement technique 
[~,t_d_pp,x_d_pp] = lsim(Pend_sys_x0, U_input, t_sim, x0d);
i_d_pp = -( K(1)*x_d_pp(:,1) + K(2)*x_d_pp(:,2) + K(3)*x_d_pp(:,3) + K(4)*x_d_pp(:,4) );
figure('Name','Closed-loop simulation to disturbance with pole-placement technique');
tiledlayout(3, 1);
nexttile;
yyaxis left
plot(t_d_pp,x_d_pp(:,1),'r','LineWidth',1.5);
title('Cart position')
xlabel('Time [s]');
ylabel('x [m]');
ylim([-0.1 0.01])
yyaxis right
plot(t_d_pp,d0*U_input(:,2),'--')
ylabel('Disturbance')

grid on
nexttile;
yyaxis left
plot(t_d_pp,rad2deg(x_d_pp(:,3)),'b','LineWidth',1.5);
title('Pendulum angle')
xlabel('Time [s]');
ylabel('\theta [deg]');
ylim([-7.5, 3])
yyaxis right
plot(t_d_pp,d0*U_input(:,2),'--')
ylabel('Disturbance')
grid on

nexttile;
plot(t_d_pp,i_d_pp,'LineWidth',1.5);
title('Current');
xlabel('Time [s]');
ylabel('i [A]');
grid on
ylim([-1.5, 1.5])

% Task 6.3
% The observability of the system was assessed by means of the MATLAB
% built-in command obsv.
Ob = obsv(A,C);
n = size(A,1);
isObservable = (rank(Ob)==n); % 1 = true
% The observer poles should be 2 to 6 times faster than the controller poles 
% to ensure the estimation error converges before the control action dominates. 
% However, placing observer poles too far in the LHP expands the filter's bandwidth, 
% amplifying high-frequency sensor noise. 
% The choice of w_O = 30 rad/s is a deliberate trade-off between transient 
% response speed and noise rejection.

% Task 6.5
% The observer poles were placed and the closed-loop response of the
% nonlinear system with friction, regulated by the dynamic compensator, was
% investigated under disturbance input. The results were finally
% illustrated in the figures reported below, together with the estimation
% error between the observed and the actual state variables.
w_O = 30;
Re_P_O = xi*w_O;
Im_P_O = w_O*sqrt(1-xi^2);
Po_1 = -Re_P_O + 1i*Im_P_O;
Po_2 = -Re_P_O - 1i*Im_P_O;
Po_3 = -30;
Po_4 = -40;
Po = [Po_1 Po_2 Po_3 Po_4];
lT = place(A',C',Po);
L = lT';
x0_real = [0; 0; 0; 0];
x0_hat = [0; 0; 0; 0];
X0_totale = [x0_real; x0_hat];
ODE_obj.ODEFcn = @(t,x) complete_system_dynamic(t, x, @Pend_input,Pend, ...
   A,B,C,K,L);
ODE_obj.InitialValue = X0_totale;
ODE_obj.Solver = 'ode45';
ODEResults_obj = solve(ODE_obj,0,10);
t_comp = ODEResults_obj.Time';
x_comp = ODEResults_obj.Solution'; % 8-components vector: first four real ones
% last four estimated ones
x_real  = x_comp(:, 1:4);
x_hat   = x_comp(:, 5:8);
error_estimation = x_real - x_hat;
i_comp = -( K(1)*x_hat(:,1) + K(2)*x_hat(:,2) + K(3)*x_hat(:,3) + K(4)*x_hat(:,4) );

figure('Name','closed-loop response to disturbance of the system with dinamic compensator ');
tiledlayout(3, 1);
nexttile;
plot(t_comp, x_real(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_comp, x_hat(:,1), 'r--', 'LineWidth', 1.5);
title('Cart position: x(t)');
ylabel('x [m]')
xlabel('Time [s]')
ylim([-0.1 0.03])
legend('Real', 'Estimated');
grid on;
nexttile;
plot(t_comp, rad2deg(x_real(:,3)), 'b', 'LineWidth', 1.5); hold on;
plot(t_comp, rad2deg(x_hat(:,3)), 'r--', 'LineWidth', 1.5);
title('Pendulum angle: x(t)');
ylabel('\theta [deg]')
xlabel('Time [s]')
legend('Real', 'Estimated');
ylim([-10, 5])
grid on;
nexttile;
plot(t_comp,i_comp,'LineWidth',1.5);
title('Current');
xlabel('Time [s]');
ylabel('i [A]');
ylim([-2 2.5])
grid on;

figure('Name','Estimation error of x and theta');
tiledlayout(2, 1);
nexttile;
plot(t_comp, error_estimation(:,3), 'k', 'LineWidth', 1.5);
title('Estimation error on the angle \theta: (Real - Estimation)','FontSize',10);
ylabel('err [deg]','FontSize',10)
xlabel('Time [s]','FontSize',10)
set(gca, 'FontSize', 10);
grid on;
nexttile;
plot(t_comp, error_estimation(:,1), 'k', 'LineWidth', 1.5);
title('Estimation error on x: (Real - Estimated)');
grid on;
ylabel('err [m]','FontSize',10)
xlabel('Time [s]','FontSize',10)
set(gca, 'FontSize', 10);

% Comparison of the system response to the disturbance with and without the observer
ODE_obj.ODEFcn = @(t,x) ideal_dynamics_without_observe(t, x, @Pend_input,Pend, ...
   K);
ODE_obj.InitialValue = x0_real;
ODE_obj.Solver = 'ode45';
ODEResults_obj = solve(ODE_obj,0,10);
t_comp_id = ODEResults_obj.Time';
x_comp_id = ODEResults_obj.Solution';

i_comp_id = -(K(1)*x_comp_id(:,1) + K(2)*x_comp_id(:,2) + ...
    K(3)*x_comp_id(:,3) + K(4)*x_comp_id(:,4));

figure('Name','Comparison system with and without observer');
tiledlayout(3,1);
nexttile;
plot(t_comp,x_real(:,1),'r','LineWidth',1.5);
hold on
plot(t_comp_id,x_comp_id(:,1),'--b','LineWidth',1.5);
grid on;
ylabel('x [m]')
xlabel('Time [s]')
ylim([-0.1 0.03])
title('Cart position')
legend('with obsv','without obsv')
nexttile;
plot(t_comp,rad2deg(x_real(:,3)),'r','LineWidth',1.5);
hold on
plot(t_comp_id,rad2deg(x_comp_id(:,3)),'--b','LineWidth',1.5);
grid on;
ylabel('\theta [deg]')
xlabel('Time [s]')
ylim([-10, 5])
title('Pendulum angle')
legend('with obsv','without obsv')
nexttile;
plot(t_comp, i_comp, 'r', 'LineWidth', 1.5);
hold on
plot(t_comp_id,i_comp_id,'--b', 'LineWidth', 1.5)
title('Current');
xlabel('Time [s]');
ylabel('i [A]');
ylim([-1.5, 2])
grid on;
legend('with obsv','without obsv')

% Task 6.6
% The analysis carried out in Task 6.5 was replicated by implementing the
% system in Simulink by means of MATLAB Function blocks.
open_system('Model_22_01_2.slx')

%% Functions

function dXdt = complete_system_dynamic(t, x, input_fun,Pend, A, B, C, K, L)

% complete system
%
% Complete system dynamics with dinamic compensator
%
% % INPUT
    % % Name            Type                Size [when applicable]
    % % t               Vector              
    % % x               Vector              
    % % input_fun       Function           
    % % Pend            Function           
    % % A               Matrix              4x4
    % % B               Matrix              4x2
    % % C               Matrix              2x4
    % % K               Vector              1x4
    % % L               Matrix              4x2
    %
    % % OUTPUT
    % % Name            Type                Size [when applicable]
    % % dXdt            Vector              8x1

   M = Pend.M;
   m = Pend.m;
   l = Pend.l;
   g = Pend.g;
   b = Pend.b;
   c = Pend.c;
   d0 = Pend.d0;
   r = Pend.r;
   kt = Pend.kt;
   am = kt/r;
  
   x_real = x(1:4);   
   x_estimation = x(5:8);   

   y = C * x_real;
 
   i = -K * x_estimation;
  
  d = input_fun(t,Pend);
  
   x2 = x_real(2);
   x3 = x_real(3);
   x4 = x_real(4);
  
   xdot2 = (1/2*m*l*cos(x3)*((m*g*l/2)*sin(x3)-b*x4+d0*d*l^2/2*cos(x3))+1/3*m*l^2* ...
   (-1/2*m*l*(x4)^2*sin(x3)-c*x2+am*i-d0*l*d))/(1/3*m*(M+m)*l^2-1/4*m^2*l^2*cos(x3)^2);
   xdot4   = ((M+m)*(m*g*l/2*sin(x3)-b*x4+d0*d*l^2/2*cos(x3))+1/2*m*l*cos(x3)*...
   (-1/2*m*l*(x4)^2*sin(x3)-c*x2+am*i-d0*l*d))/(1/3*m*(M+m)*l^2-1/4*m^2*l^2*cos(x3)^2);
  
   xdot_real = zeros(4,1);
   xdot_real(1) = x_real(2);                
   xdot_real(2) = xdot2;   
   xdot_real(3) = x_real(4);                
   xdot_real(4) = xdot4;      

   xdot_estimation = A * x_estimation + B(:,1) * i + L * (y - C * x_estimation);
   dXdt = [xdot_real; xdot_estimation];
 
  
end


function dXdt = ideal_dynamics_without_observe(t, x, input_fun, Pend, K)

%
% ideal system without observer
%
% % INPUT
    % % Name            Type                Size [when applicable]
    % % t               Vector              
    % % x               Vector              
    % % input_fun       Function           
    % % Pend            Function           
    % % K               Vector              1x4
    %
    % % OUTPUT
    % % Name            Type                Size [when applicable]
    % % dXdt            Vector              4x1

   M = Pend.M; m = Pend.m; l = Pend.l; g = Pend.g;
   b = Pend.b; c = Pend.c; d0 = Pend.d0; r = Pend.r; kt = Pend.kt; am = kt/r;
  
   i = -K * x;
  
   d = input_fun(t, Pend);
  
   x2 = x(2);
   x3 = x(3);
   x4 = x(4);
  
   xdot2 = (1/2*m*l*cos(x3)*((m*g*l/2)*sin(x3)-b*x4+d0*d*l^2/2*cos(x3))+1/3*m*l^2* ...
   (-1/2*m*l*(x4)^2*sin(x3)-c*x2+am*i-d0*l*d))/(1/3*m*(M+m)*l^2-1/4*m^2*l^2*cos(x3)^2);
   
   xdot4   = ((M+m)*(m*g*l/2*sin(x3)-b*x4+d0*d*l^2/2*cos(x3))+1/2*m*l*cos(x3)*...
   (-1/2*m*l*(x4)^2*sin(x3)-c*x2+am*i-d0*l*d))/(1/3*m*(M+m)*l^2-1/4*m^2*l^2*cos(x3)^2);
  
   dXdt = [x(2); xdot2; x(4); xdot4];
end


function Pend = Pend_parameters()

% Pend_parameters imposed by the tasks, the Pend.obj is a structure
% containing the parameters

 M = 0.57;
 rho = 0.36;
 l = 0.64;
 m = rho*l;
 Ig = m*l^2/12;
 g = 9.81;
 c = 0.1;
 b = 0.005;
 r = 0.02;
 kt = 0.05;
 d0 = 0.3;
  Pend.M = M;
  Pend.rho = rho;
  Pend.l = l;
  Pend.m = m;
  Pend.Ig = Ig;
  Pend.g = g;
  Pend.c = c;
  Pend.b = b;
  Pend.r = r;
  Pend.kt = kt;
  Pend.d0 = d0;
end


function xdot = Pend_f(t,x,input_fun,Pend,c,b,kp,kd)

% Pend_f
%
% Non-linear system equations wrote in state space model
%
% % INPUT
    % % Name            Type                Size [when applicable]
    % % t               Vector              
    % % x               Vector              
    % % input_fun       Function           
    % % Pend            Function           
    % % c               Scalar              1x1
    % % b               Scalar              1x1
    % % kp              Scalar              1x1
    % % kd              Scalar              1x1
    %
    % % OUTPUT
    % % Name            Type                Size [when applicable]
    % % xdot            Vector              4x1

kt = Pend.kt;
r = Pend.r;
d0 = Pend.d0;
g = Pend.g;
m = Pend.m;
M = Pend.M;
l = Pend.l;
am = kt/r;

x2 = x(2);
x3 = x(3);
x4 = x(4);
d = input_fun(t,Pend);
i = -kp*x3-kd*x4;
xdot1 = x2;
xdot2 = (1/2*m*l*cos(x3)*((m*g*l/2)*sin(x3)-b*x4+d0*d*l^2/2*cos(x3))+1/3*m*l^2* ...
   (-1/2*m*l*(x4)^2*sin(x3)-c*x2+am*i-d0*l*d))/(1/3*m*(M+m)*l^2-1/4*m^2*l^2*cos(x3)^2);
xdot3 = x4;
xdot4 = ((M+m)*(m*g*l/2*sin(x3)-b*x4+d0*d*l^2/2*cos(x3))+1/2*m*l*cos(x3)*...
   (-1/2*m*l*(x4)^2*sin(x3)-c*x2+am*i-d0*l*d))/(1/3*m*(M+m)*l^2-1/4*m^2*l^2*cos(x3)^2);
xdot = [xdot1;
       xdot2;
       xdot3;
       xdot4];
end


function xdot = Pend_f_5(t,x,input_fun,Pend,c,b,kp,kd,ki)

% Pend_f_5
%
% Augmented State Feedback formulation for the integral action:
% To apply the PID controller to the non-linear state-space model, the system state
% is augmented with x5, which represents the integral of the angle tracking error.
%
% % INPUT
    % % Name            Type                Size [when applicable]
    % % t               Vector              
    % % x               Vector              
    % % input_fun       Function           
    % % Pend            Function           
    % % c               Scalar              1x1
    % % b               Scalar              1x1
    % % kp              Scalar              1x1
    % % kd              Scalar              1x1
    % % ki              Scalar              1x1
    %
    % % OUTPUT
    % % Name            Type                Size [when applicable]
    % % xdot            Vector              4x1

kt = Pend.kt;
r = Pend.r;
d0 = Pend.d0;
g = Pend.g;
m = Pend.m;
M = Pend.M;
l = Pend.l;
am = kt/r;

x2 = x(2);
x3 = x(3);
x4 = x(4);
x5 = x(5);
d = input_fun(t,Pend);
i = -kp*x3-kd*x4-ki*x5;
xdot1 = x2;
xdot2 = (1/2*m*l*cos(x3)*((m*g*l/2)*sin(x3)-b*x4+d0*d*l^2/2*cos(x3))+1/3*m*l^2* ...
   (-1/2*m*l*(x4)^2*sin(x3)-c*x2+am*i-d0*l*d))/(1/3*m*(M+m)*l^2-1/4*m^2*l^2*cos(x3)^2);
xdot3 = x4;
xdot4 = ((M+m)*(m*g*l/2*sin(x3)-b*x4+d0*d*l^2/2*cos(x3))+1/2*m*l*cos(x3)*...
   (-1/2*m*l*(x4)^2*sin(x3)-c*x2+am*i-d0*l*d))/(1/3*m*(M+m)*l^2-1/4*m^2*l^2*cos(x3)^2);
xdot5 = x3;
xdot = [xdot1;
       xdot2;
       xdot3;
       xdot4
       xdot5];
end


function  d = Pend_input(t,Pend)

% Pend_input
%
% Pend_disturbance function
%
% % INPUT
    % % Name            Type                Size [when applicable]
    % % t               Vector                       
    % % Pend            Function           
    %
    % % OUTPUT
    % % Name            Type                Size [when applicable]
    % % d               Vector
T1 = Pend.T1;
T2 = Pend.T2;
d_max = Pend.d_max;
  if t < T1
      d = 0;
  elseif t<T2
      d = d_max;
  else
      d = 0;
  end
end


function [A,B,C,D] = Pend_sys(Pend,c,b)

% Pend_sys
%
% State space model matrices for the frictionless linear model
%
% % INPUT
    % % Name            Type                Size [when applicable]          
    % % Pend            Function           
    % % c               Scalar              1x1
    % % b               Scalar              1x1
    %
    % % OUTPUT
    % % Name            Type                Size [when applicable]
    % % A               Matrix              4x4
    % % B               Matrix              4x2
    % % C               Matrix              2x4
    % % d               Matrix              1x1

kt = Pend.kt;
r = Pend.r;
d0 = Pend.d0;
g = Pend.g;
m = Pend.m;
M = Pend.M;
l = Pend.l;
am = kt/r;
I1 = m*l/2;
I2 = m*l^2/3;
a0 = d0*l;
a1 = d0*l^2/2;

A = [0 1 0 0;
   0 -c*I2/(I2*(M+m)-I1^2) I1^2*g/(I2*(M+m)-I1^2) -b*I1/(I2*(M+m)-I1^2);
   0 0 0 1;
   0 -c*I1/(I2*(M+m)-I1^2) (M+m)*I1*g/(I2*(M+m)-I1^2) -(M+m)*b/(I2*(M+m)-I1^2)];
B = [0 0;
   I2*am/(I2*(M+m)-I1^2) (I1*a1-I2*a0)/(I2*(M+m)-I1^2);
   0 0;
   I1*am/(I2*(M+m)-I1^2) ((M+m)*a1 -I1*a0)/(I2*(M+m)-I1^2)];
C = [1 0 0 0;
   0 0 1 0];
D = 0;
end


function x_zdot = Pend_z(t,xl,input_fun,Pend,c,b)

% Pend_linearized. 
% 
% linearized motion equations wrote in state-space model;
% the l stands for linearized.
%
% % INPUT
    % % Name            Type                Size [when applicable]
    % % t               Vector              
    % % xl              Vector              
    % % input_fun       Function           
    % % Pend            Function           
    % % c               Scalar              1x1
    % % b               Scalar              1x1
    %
    % % OUTPUT
    % % Name            Type                Size [when applicable]
    % % x_zdot          Vector              4x1

d0 = Pend.d0;
g = Pend.g;
m = Pend.m;
M = Pend.M;
l = Pend.l;
I1 = m*l/2;
I2 = m*l^2/3;
a0 = d0*l;
a1 = d0*l^2/2;

xl2 = xl(2);
xl3 = xl(3);
xl4 = xl(4);
d = input_fun(t,Pend);
xldot1 = xl2;
xldot2 = (I1*(a1*d+I1*g*xl3-b*xl4)+I2*(-a0*d-c*xl2))/(I2*(M+m)-I1^2);
xldot3 = xl4;
xldot4 = ((M+m)*(a1*d+I1*g*xl3-b*xl4)+I1*(-a0*d-c*xl2))/(I2*(M+m)-I1^2);
x_zdot = [xldot1;
       xldot2;
       xldot3;
       xldot4];
end


function [t, x] = forward_euler(ode_fun, t_span, x0, Pend)

% forward euler
%
% % INPUT
    % % Name            Type                Size [when applicable]
    % % ode_fun         Function              
    % % t_span          Vector              1x10001
    % % x0              Vector              4x1
    % % Pend            Function           
    %
    % % OUTPUT
    % % Name            Type                Size [when applicable]
    % % t               Vector              10001x1
    % % x               Matrix              10001x4
  
   t = t_span(:); 
   N = length(t);
   num_stati = length(x0);
  
   x = zeros(N, num_stati);
   x(1, :) = x0';
  
   input_zero = @(t_val, Pend_val) 0;
  
   for k = 1:(N-1)
       h = t(k+1) - t(k);

       x_dot = ode_fun(t(k), x(k, :)', input_zero, Pend);
       x(k+1, :) = x(k, :) + (h * x_dot)';
   end
end


function [t,x] = ode_Euler_b(fun,tspan,x_t0,h)

% Backward euler function, uses fsolve to solve the implicit equations
% 
% % INPUT
    % % Name            Type                Size [when applicable]
    % % fun             Function              
    % % t_span          Vector              1x2
    % % x0_t0           Vector              4x1
    % % Pend            Function    
    % % h               Scalar              1x1
    %
    % % OUTPUT
    % % Name            Type                Size [when applicable]
    % % t               Vector              1x10001
    % % x               Matrix              10001x4

t0 = tspan(1);
tf = tspan(2);
t = t0 : h : tf;
N_h = (tf-t0)/h;
x(:,1) = x_t0;
options = optimoptions('fsolve','Display','none');
for n = 1:N_h
   x(:,n+1) = fsolve(@(X) X - x(:,n) - h*feval(fun,t(n+1),X),x(:,n),options);
end
end
