% Function to get the vector state of a DER for the deviation system
% The deviation system is covered Lewis, Optimal Control, 2012 (page
% 317-318)
% Assumptions: Only 2 states.
% The function is used after simulink file is run.

function x_g = get_x_g_der(out_sim, time_interval)
    x1 = out_sim.Data(:,1);
    x1 = x1(time_interval);
    x2 = out_sim.Data(:,2);
    x2 = x2(time_interval);
    x1_g = x1(:,1) - x1(end,1);
    x2_g = x2(:,1) - x2(end,1);

    x_g = [x1_g x2_g]';
end


% You may calculate the end value by applying formula:
% For two DERs (PV1 and BESS2) and with initial input control condition 5e6 each one.
% The reference is the sume preq + load = 25e6
% The formula was checked numerically by comparing the resulted ones with the one of the Nash model simulation
% There are certain discrepancies when it is compared to the Nash grid (with full schemes)
% G = [0; 0; 0; 0; 1];
% Ac = A_aug + B_aug*([PV1.NE_kp PV1.NE_ki;BESS2.NE_kp BESS2.NE_ki]+[5e6;5e6]);
% Bc = G;
% xsss = -(A_aug + B_aug*[PV1.NE_kp PV1.NE_ki;BESS2.NE_kp BESS2.NE_ki])^(-1)*(B_aug*[5e6;5e6]+G*25e6)

