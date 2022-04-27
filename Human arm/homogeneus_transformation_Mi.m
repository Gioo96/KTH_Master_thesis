function [M1_W, M2_W, M3_W, M4_W, M5_W, M6_W, M7_W, M8_W, M9_W] = homogeneus_transformation_Mi(marker, output, i)

syms q1 q2 q3 q4 q5 q6 q7;
%% Shoulder

M1_W = double(subs(marker.TW_M1, {q1 q2 q3}, {output.q.signals.values(i,1) output.q.signals.values(i,2) output.q.signals.values(i,3)})) * (output.M1_M1.signals.values(i,:))';
M2_W = double(subs(marker.TW_M2, {q1 q2 q3}, {output.q.signals.values(i,1) output.q.signals.values(i,2) output.q.signals.values(i,3)})) * (output.M2_M2.signals.values(i,:))';
M3_W = double(subs(marker.TW_M3, {q1 q2 q3}, {output.q.signals.values(i,1) output.q.signals.values(i,2) output.q.signals.values(i,3)})) * (output.M3_M3.signals.values(i,:))';
M4_W = double(subs(marker.TW_M4, {q1 q2 q3}, {output.q.signals.values(i,1) output.q.signals.values(i,2) output.q.signals.values(i,3)})) * (output.M4_M4.signals.values(i,:))';
%% ForeArm

M5_W = double(subs(marker.TW_M5, {q1 q2 q3 q4}, {output.q.signals.values(i,1) output.q.signals.values(i,2) output.q.signals.values(i,3) output.q.signals.values(i,4)})) * (output.M5_M5.signals.values(i,:))';
M6_W = double(subs(marker.TW_M6, {q1 q2 q3 q4}, {output.q.signals.values(i,1) output.q.signals.values(i,2) output.q.signals.values(i,3) output.q.signals.values(i,4)})) * (output.M6_M6.signals.values(i,:))';
M7_W = double(subs(marker.TW_M7, {q1 q2 q3 q4}, {output.q.signals.values(i,1) output.q.signals.values(i,2) output.q.signals.values(i,3) output.q.signals.values(i,4)})) * (output.M7_M7.signals.values(i,:))';

%% Hand

M8_W = double(subs(marker.TW_M8, {q1 q2 q3 q4 q5 q6 q7}, {output.q.signals.values(i,1) output.q.signals.values(i,2) output.q.signals.values(i,3) output.q.signals.values(i,4) output.q.signals.values(i,5) output.q.signals.values(i,6) output.q.signals.values(i,7)})) * (output.M8_M8.signals.values(i,:))';
M9_W = double(subs(marker.TW_M9, {q1 q2 q3 q4 q5 q6 q7}, {output.q.signals.values(i,1) output.q.signals.values(i,2) output.q.signals.values(i,3) output.q.signals.values(i,4) output.q.signals.values(i,5) output.q.signals.values(i,6) output.q.signals.values(i,7)})) * (output.M9_M9.signals.values(i,:))';

end