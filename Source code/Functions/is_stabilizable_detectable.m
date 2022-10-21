function [is_stabilizable, is_detectable] = is_stabilizable_detectable(A, B, C)

%% Description

% is_stabilizable_detectable check if the pair (A, B) is stabilizable and
% if (A, C) is detectable

[is_controllable, is_observable] = is_controllable_observable(A, B, C);

%% STABILIZABILITY
% (A, B) CONTROLLABLE
if (is_controllable)

    is_stabilizable = true;

% (A, B) NOT CONTROLLABLE
else

    [Abar, Bbar, Cbar, T, k] = ctrbf(A, B, C);
    c_matrix = ctrb(Abar, Bbar);
    rank_c = rank(c_matrix);
    rank_nc  = size(Abar, 1)  - rank_c;

    % Check STABILIZABILITY
    eigs_nc = eig(Abar(1:rank_nc, 1:rank_nc));
    is_stabilizable = true;
    for i = 1 : size(eigs_nc, 1)

        if (abs(eigs_nc(i)) >= 1)

            is_stabilizable = false;
        end
    end
end

%% DETECTABILITY
% (A, C) OBSERVABLE
if (is_observable)

    is_detectable = true;
    eigs_no = [];

% (A, C) NOT OBSERVABLE
else

    [Abar, Bbar, Cbar, T, k] = obsvf(A, B, C);
    o_matrix = obsv(Abar, Cbar);
    rank_o = rank(o_matrix);
    rank_no  = size(Abar, 1)  - rank_o;

    % Check DETECTABILITY
    eigs_no = eig(Abar(1:rank_no, 1:rank_no));
    is_detectable = true;
    for i = 1 : size(eigs_no, 1)

        if (abs(eigs_no(i)) >= 0.99999)

            is_detectable = false;
        end
    end

end