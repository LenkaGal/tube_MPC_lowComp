clear all, close all;
load tube_empc_flexy
load tube19
f_b = out.y(:,2)-ys; %flex bend in deviation form
u_prev = out.u(:,2)-us; %previous control action in deviation form
time = [];
for i = 1:length(f_b)
    tstart = tic;
    [ u, feasible ] = eMPC.evaluate(f_b(i),'u.previous', u_prev(i));
    time = [time; toc(tstart)];
end
av = mean(time) %average computational time
[m,i] = max(time) %worst-case computational time

