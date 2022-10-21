function [is_controllable, is_observable] = is_controllable_observable(A, B, C)

%% Description

% is_controllable_observable check if the pair (A, B) is controllable and
% if (A, C) is observable

% Controllability matrix
c_matrix = ctrb(A, B);

% Observability matrix
o_matrix = obsv(A, C);

rank_c = rank(c_matrix);
rank_nc = size(A, 1) - rank_c;
rank_o = rank(o_matrix);
rank_no = size(A, 1) - rank_o;

% (A, B) NOT CONTROLLABLE
if (rank_nc > 0)

    is_controllable = false;

% (A, B) CONTROLLABLE
else

    is_controllable = true;
end

% (A, C) NOT OBSERVABLE
if (rank_no > 0)

    is_observable = false;
    
% (A, C) OBSERVABLE
else

    is_observable = true;
end

end