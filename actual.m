clc
clear variables
% close all

global u_in

%% Time interval details and initial conditions
t_final = 15; % in seconds
t_interval = 0.01; % in seconds
init_val = [0 0 0 0 0 0 0 0 0 0 0 0];

m = 1.535; %mass of drone
g = 9.81; %gravity

%Inertia values
Ixx= 0.029125;
Iyy= 0.029125;
Izz= 0.055225; 

kt=0.0;
kr=0.0;

cnt_f = round(t_final/t_interval);
zero_M = zeros(1,1+round(t_final/t_interval));
T2 = zero_M;
init_val_M = zeros(1+round(t_final/t_interval),size(init_val,12));

%% Controls
k_p = 20; k_d = 5;
e_prev = 0;
e_ = 0;
x_ref_ = 10;

%%Simulation
for i = 0:t_interval:t_final
    t_current = [i*t_interval, (i+1)*t_interval];
    cnt_ = round((i + t_interval)/t_interval);
    x_1 = init_val(:,1); 
    x_2 = init_val(:,2); 
    x_3 = init_val(:,3);
    x_4 = init_val(:,4);
    x_5 = init_val(:,5);
    x_6 = init_val(:,6);
    x_7 = init_val(:,7);
    x_8 = init_val(:,8);
    x_9 = init_val(:,9);
    x_10 = init_val(:,10);
    x_11 = init_val(:,11);
    x_12 = init_val(:,12);

    e_ = x_ref_ - x_1;
    e_der = (e_ - e_prev)/t_interval;
    u_in = k_p*e_ + k_d*e_der;

    [t, y] = ode45(@(t, y) droneDynamics(t, y, u_in,m,g,Ixx,Iyy,Izz,kt,kr), t_current,init_val);
    init_val = Y(2,:);
    T2(cnt_) = t(1,:);init_val_M(cnt_,:) = init_val;
    e_prev = e_;
end

x_1M = init_val_M(:,1);
x_2M = init_val_M(:,2);
x_3M = init_val_M(:,3);
x_4M = init_val_M(:,4);
x_5M = init_val_M(:,5);
x_6M = init_val_M(:,6);
x_7M = init_val_M(:,7);
x_8M = init_val_M(:,8);
x_9M = init_val_M(:,9);
x_10M = init_val_M(:,10);
x_11M = init_val_M(:,11);
x_12M = init_val_M(:,12);

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