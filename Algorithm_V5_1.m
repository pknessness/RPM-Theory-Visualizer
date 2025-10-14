clc; clear; close all;

%% PARAMETERS
g = 9.81;                        % gravitational acceleration (m/s^2)
r_worst_case = 6*0.0254;         % worst-case radius (m)
runtime = 24;                    % runtime in hours (for DT calculation)
N_SIM = 10000;                    % number of simulation steps
LENGTH_R = 1e5;                  % number of random candidate points (reduced for efficiency)
MAX_ANGLE_DEG = 30;              % maximum allowed deviation angle (degrees)
MAX_COS_ANGLE = cosd(MAX_ANGLE_DEG);
DS = 0.1;                        % fixed step length (m)
NUM_DIST_SEARCH = 200;           % candidate pool size per iteration
DT = runtime*60*60 / N_SIM;      % time per simulation step
useOptimization = true;          % toggle for candidate selection optimization

%% TRAJECTORY GENERATION
% Generate random candidate points on the unit sphere
r = randn(3, LENGTH_R);
r = r ./ vecnorm(r);

% Initialize the trajectory
first_point = [0; 1; 0];
distances0 = vecnorm(r - first_point, 2, 1);
[~, idx0] = mink(distances0, 11);
idx0 = idx0(2:end);
second_point = r(:, idx0(randi(numel(idx0))));
first_vector = (second_point - first_point);
first_vector = first_vector / norm(first_vector);
second_point = first_point + DS * first_vector;
second_point = second_point / norm(second_point);

trajectory = zeros(3, N_SIM+2);
trajectory(:,1) = first_point;
trajectory(:,2) = second_point;
candidateOrientations = zeros(3, N_SIM+1);
candidateOrientations(:,1) = first_vector;

previous_point = second_point;
last_vector    = first_vector;

% Precompute acceleration scaling factors
accel_vector_g = [0; -g; 0];
if r_worst_case > 0
    a_centripetal_factor = (DS * r_worst_case / DT)^2 / r_worst_case;
    a_tangential_factor  = (DS * r_worst_case / DT)^2;
else
    a_centripetal_factor = 0;
    a_tangential_factor  = 0;
end

% Progress marker setup
pctStep = max(floor(N_SIM/100),1);
fprintf('Progress:   0%%');

% Simulation loop
for sim = 1:N_SIM
    % find nearest candidates
    vecs_to_cand = r - previous_point;
    dists = vecnorm(vecs_to_cand, 2, 1);
    [~, tmpIdx] = mink(dists, NUM_DIST_SEARCH+1);
    tmpIdx = tmpIdx(2:end);

    cand_vecs = r(:, tmpIdx);
    deltas   = cand_vecs - previous_point;
    norms    = vecnorm(deltas,2,1);
    cosA     = dot(deltas, repmat(last_vector,1,NUM_DIST_SEARCH),1) ./ (norms * norm(last_vector));

    valid = tmpIdx(cosA >= MAX_COS_ANGLE);
    if isempty(valid)
        warning('No valid candidate points found at step %d.', sim);
        break;
    end

    if useOptimization
        nV = numel(valid);
        errs = zeros(1,nV);
        for k = 1:nV
            pt = r(:, valid(k));
            dir = (pt - previous_point); dir = dir/norm(dir);
            nxt = previous_point + DS*dir; nxt = nxt/norm(nxt);
            a_cent = a_centripetal_factor * (-nxt);
            a_tang = a_tangential_factor  * dir;
            errs(k) = abs(dot(nxt, accel_vector_g + a_cent + a_tang));
        end
        [~, best] = min(errs);
        chosen = valid(best);
    else
        chosen = valid(randi(numel(valid)));
    end

    dir = (r(:,chosen) - previous_point);
    dir = dir / norm(dir);
    nxt = previous_point + DS*dir;
    nxt = nxt / norm(nxt);

    trajectory(:, sim+2)        = nxt;
    candidateOrientations(:,sim+1) = dir;
    last_vector = dir;
    previous_point = nxt;

    % update progress
    if mod(sim,pctStep)==0
        fprintf('\rProgress: %3d%%', round(100*sim/N_SIM));
    end
end

% save once at end
save('trajectory.mat','trajectory','candidateOrientations','r');
fprintf('\n');

%% EFFECTIVE-g CALCULATION (projection method)
effG_inst = zeros(1,N_SIM);
for i = 1:N_SIM
    n_i = trajectory(:,i+2);
    t_i = candidateOrientations(:,i+1);
    a_cent = a_centripetal_factor * (-n_i);
    a_tang = a_tangential_factor  * t_i;
    a_tot  = accel_vector_g + a_cent + a_tang;
    effG_inst(i) = dot(n_i,a_tot);
end
effectiveG = mean(abs(effG_inst));
normG      = effectiveG / g;
fprintf('→ Effective‑g (projection): %.5f m/s² (%.5f g)\n', effectiveG, normG);

%% VECTOR‑AVERAGING EFFECTIVE‑g
g_sample = zeros(3,N_SIM);
for i = 1:N_SIM
    n_i = trajectory(:,i+2);
    t_i = candidateOrientations(:,i+1);
    % Gram–Schmidt to ensure orthonormal triad
    t_i = t_i - (n_i'*t_i)*n_i;
    t_i = t_i / norm(t_i);
    b_i = cross(n_i,t_i); b_i = b_i/norm(b_i);
    R_i = [t_i, b_i, n_i];
    g_sample(:,i) = R_i' * accel_vector_g;
end
g_eff_vecavg = norm(mean(g_sample,2));
fprintf('→ Effective‑g (vector‑avg): %.5f m/s² (%.5f g)\n', ...
        g_eff_vecavg, g_eff_vecavg/g);

%% FIGURE 1: Full Sphere with Trajectory & Cones
figure;
[sx,sy,sz] = sphere(50);
surf(sx,sy,sz,'FaceAlpha',0.1,'EdgeColor','none');
colormap([0.8 0.8 0.8]); hold on;
hSc = scatter3(r(1,1:500:end),r(2,1:500:end),r(3,1:500:end),5,'b','filled');
if isprop(hSc,'MarkerFaceAlpha'); hSc.MarkerFaceAlpha=0.3; end
nShow = min(500, size(trajectory,2)-1);
for i = 1:nShow
    p1 = trajectory(:,i); p2 = trajectory(:,i+1);
    plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'r-','LineWidth',2);
    % draw cone region
    apex = p1; orient = candidateOrientations(:,i);
    len = DS; rcone = len*tan(deg2rad(MAX_ANGLE_DEG));
    B = null(orient');
    u1=B(:,1); u2=B(:,2);
    theta = linspace(0,2*pi,50);
    pts = apex + orient*len + rcone*(u1*cos(theta)+u2*sin(theta));
    plot3(pts(1,:),pts(2,:),pts(3,:),'m-','LineWidth',1.2);
    for j=1:10:length(theta)
        plot3([apex(1) pts(1,j)],[apex(2) pts(2,j)],[apex(3) pts(3,j)],'m--');
    end
end
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Full Sphere with First 50 Trajectory Points & Cones');
axis equal; grid on; hold off;

%% FIGURE 2: RPM Frame Schematic
% if size(trajectory,2)>=51
%     target = trajectory(:,51);
% else
%     target = trajectory(:,end);
% end
% [az,el,~] = cart2sph(target(1),target(2),target(3));
% figure; hold on; grid on; axis equal;
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title('RPM Frames & Position Vector');
% R_out = [cos(az) -sin(az) 0; sin(az) cos(az) 0; 0 0 1];
% drawSquareFrame([0;0;0],0.4,0.05,0.05,R_out,[0.5 0.5 0.5]);
% R_in  = R_out * [1 0 0; 0 cos(el) -sin(el); 0 sin(el) cos(el)];
% drawSquareFrame([0;0;0],0.3,0.05,0.05,R_in,[0.5 0.5 0.5]);
% n_in = R_in*[0;0;1];
% quiver3(0,0,0,n_in(1)*0.3,n_in(2)*0.3,n_in(3)*0.3,'b','LineWidth',2,'MaxHeadSize',2);
% hold off;
