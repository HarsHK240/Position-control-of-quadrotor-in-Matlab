clc
clear variables
% close all

global u_in

%% Time interval details and initial conditions
t_final = 9; % in seconds
t_interval = 0.01; % in seconds
init_val = [0 0];

cnt_f = round(t_final/t_interval);
zero_M = zeros(1,1+round(t_final/t_interval));
T2 = zero_M;
init_val_M = zeros(1+round(t_final/t_interval),size(init_val,2));

%% Controls
k_p = 20; k_d = 5;
e_prev = 0;
e_ = 0;
ref_ = 1;

%% Simulation
for i = 0:t_interval:t_final
    cnt_ = round((i + t_interval)/t_interval);
    x_1 = init_val(:,1); x_2 = init_val(:,2); 
    % System Inputs
    
    e_ = ref_ - x_1;
    e_der = (e_ - e_prev)/t_interval;
    u_in = k_p*e_ + k_d*e_der;
    % Future values from model equations using ODE45
    [t,Y] = ode45(@second_order_ODE,[i,t_interval+i,(2*t_interval)+i],init_val);
    T2(cnt_) = t(1,:);
    init_val = Y(2,:);
    init_val_M(cnt_,:) = init_val;
    e_prev = e_;
end

x_1M = init_val_M(:,1); x_2M = init_val_M(:,2);

%% Plots
lw_ = 2; ax_lw = 1.5;
fs_1 = 16; fs_3 = 18;
path_lw = 10; del_ = 1;

figure_xy = figure('Position', [40, 60, 450, 450]);
plot(T2,x_1M(1:del_:end),'k','Linewidth',lw_);
grid on
set(gca,'GridLineStyle','-')
ax1 = gca;
ax1.FontSize = fs_1;
ax1.XColor = 'black';
ax1.YColor = 'black';
set(ax1,'linewidth',ax_lw)
axis square
xlabel('$$t$$, s','Interpreter','Latex','Fontsize',fs_3)
ylabel('$$y$$, m','Interpreter','Latex','Fontsize',fs_3)