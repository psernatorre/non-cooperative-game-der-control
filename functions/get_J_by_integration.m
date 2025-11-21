% Function to compute the individual performance metric of a player in a
% non-cooperative linear quadratic differential game
% The individual performance metric is the one related to the deviation
% system.
% tin : lower bound of the integral
% tfi : upper bound of the integral
% Ts : time step
% Q :  state-cost weighted matrix  
% R :  control weighted matrix
% x :  state vector (nstates, position in the time)
% u :  control action (1, position in the time)
% Note 1: The trapz function requies that the size (time axis) of state 
% vector must coincide with integral domain. 
% By P.Serna-Torre (2023)

function J = get_J_by_integration(tin, tfin, Ts, Q, R, x, u)

    int_domain = tin:Ts:(tfin-Ts); % assuming same spacing over the time vector, and integration until one-to-last element
    n_steps= size(x,2);
    sum = zeros(n_steps,1);
    for t=1:1:n_steps
        x_at_t = x(:,t);
        u_at_t = u(:,t);
        sum(t,1) = x_at_t'*Q*x_at_t + R*u_at_t^2;
    end
    J = trapz(int_domain,sum);

end