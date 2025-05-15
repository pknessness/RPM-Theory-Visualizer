hold on
%T =
%readmatrix("logs/log_14-05-2025_04_59_22_V_ANSH_30.0ms_1deg_0.733rads.csv");%an hour or something
%T = readmatrix("logs/log_14-05-2025_12_57_44_V_JONMODIFIED_600s_0.733rads_MAN.csv"); % jon's 24hr profile
T = readmatrix("logs/log_14-05-2025_15_32_53_V_ANSH_30.0ms_1deg_0.733rads_MAN.csv"); % my 24hr profile


n = 500 %Pick every n points

X2 = T(1:n:end,1);
Y2 = T(1:n:end,7);

Y2_b = movmean(Y2,15);

plot(X2, Y2_b, 'r-', 'LineWidth', 2);
%plot(X2, Y_intsd, 'g-', 'LineWidth', 2);

grid on;
title('Effective Accel vs. Time', 'FontSize', 20);
xlabel('Time', 'FontSize', 20);
ylabel('Effective Accel (G)', 'FontSize', 20);
yscale log %comment this out to graph in not-log scale

%ylim([0 0.1])