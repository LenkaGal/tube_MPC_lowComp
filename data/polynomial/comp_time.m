clear all, close all;
load opti_empc.mat %empc
load poly_empc.mat %eMPCsimple

load poly3
yp = out.y(:,2)-ys; %polynomial flex bend in deviation form

load opti7
yo = out.y(:,2)-ys; %optimal flex bend in deviation form

%% optimal
time_opti = [];
for i = 1:length(yo)
    tstart = tic;
    u = empc.evaluate(yo(i));
    time_opti = [time_opti; toc(tstart)];
end
disp('Optimal eMPC')
av_time = mean(time_opti) %average computational time
[max_time,i] = max(time_opti) %worst-case computational time

%% polynomial
time_poly = [];
for i = 1:length(yp)
    tstart = tic;
    u = eMPCsimple.evaluate(yp(i));
    time_poly = [time_poly; toc(tstart)];
end
disp('Polynomial eMPC')
av_time = mean(time_poly) %average computational time
[max_time,i] = max(time_poly) %worst-case computational time

