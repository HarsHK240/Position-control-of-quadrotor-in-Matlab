function xdot = second_order_ODE(t,x)

global u_in;

second_order_ODE_param

xdot = [x(2);
        -2*zeta_*omega_*x(2)-omega_*omega_*x(1)+omega_*omega_*u_in];