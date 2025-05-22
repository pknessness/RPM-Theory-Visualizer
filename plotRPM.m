hold on
%T =
%readmatrix("logs/log_14-05-2025_04_59_22_V_ANSH_30.0ms_1deg_0.733rads.csv");%an hour or something
%TJonM = readmatrix("logs/log_14-05-2025_12_57_44_V_JONMODIFIED_600s_0.733rads_MAN.csv"); % jon mdified 24hr profile
%TAnsh = readmatrix("logs/log_14-05-2025_15_32_53_V_ANSH_30.0ms_1deg_0.733rads_MAN.csv"); % my 24hr profile
TJon = readmatrix("logs/log_14-05-2025_19_05_38_V_JON.csv"); % Jon 24hr profile
TJon2 = readmatrix("logs/log_19-05-2025_12_58_00_V_JON-1.57RADs-.csv"); % Jon 24hr profile with 1.57Rad/s
%TJon3 = readmatrix("logs/log_19-05-2025_13_06_33_V_JON-3.14159RADs-.csv"); % Jon 24hr profile with 3.14Rad/s

n = 5000 %Pick every n points

XJonM = TJonM(1:n:end,1);
YJonM = TJonM(1:n:end,7);
YJonM_b = movmean(YJonM,5);
%plot(XJonM, YJonM_b, 'r-', 'LineWidth', 1);

XAnsh = TAnsh(1:n:end,1);
YAnsh = TAnsh(1:n:end,7);
YAnsh_b = movmean(YAnsh,5);
%plot(XAnsh, YAnsh_b, 'g-', 'LineWidth', 1);

XJon = TJon(1:n:end,1);
YJon = TJon(1:n:end,7);
YJon_b = movmean(YJon,5);
plot(XJon, YJon_b, 'b-', 'LineWidth', 1);

XJon2 = TJon2(1:n:end,1);
YJon2 = TJon2(1:n:end,7);
YJon2_b = movmean(YJon2,5);
plot(XJon2, YJon2_b, 'c-', 'LineWidth', 1);

XJon3 = TJon3(1:n:end,1);
YJon3 = TJon3(1:n:end,7);
YJon3_b = movmean(YJon3,5);
%plot(XJon3, YJon3_b, 'm-', 'LineWidth', 1);

grid on;
title('Effective Accel vs. Time', 'FontSize', 20);
xlabel('Time', 'FontSize', 20);
ylabel('Effective Accel (G)', 'FontSize', 20);
yscale log %comment this out to graph in not-log scale
legend('Incommensurable Velocity (7RPM)', 'Incommensurable Velocity (15RPM)')
%legend('Continual Turning Velocity', 'Simplified Random Direction', 'Incommensurable Velocity', 'Incommensurable Velocity (1.57Rad/s)', 'Incommensurable Velocity (3.14Rad/s)')