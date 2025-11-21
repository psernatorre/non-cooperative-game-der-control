% Function to get the 1-dim vector of the deviation system
% It can be used for w, and u.
% The deviation system is covered Lewis, Optimal Control, 2012 (page
% 317-318)
% Assumptions: Only 2 states.
% The function is used after simulink file is run.


function w_g = get_w_g(out_sim, time_interval)
    w = out_sim.Data;
    w = w(time_interval);
    w_g = w(:,1) - w(end,1);
    w_g = w_g';
end