    %% Full Standalone Trajectory Generation and Visualization Script
clc; clear; close all;

%% PARAMETERS
% Global parameters
g = 9.81;                        % gravitational acceleration (m/s^2)
r_worst_case = 6*0.0254;           % worst-case radius (m)
runtime = 10;                    % runtime in hours (for DT calculation)
N_SIM = 1000;                    % number of simulation steps
LENGTH_R = 10000000;             % number of random candidate points
MAX_ANGLE_DEG = 30;              % maximum allowed deviation angle (degrees)
MAX_COS_ANGLE = cosd(MAX_ANGLE_DEG);
DS = 0.1;                        % fixed step length (m)
NUM_DIST_SEARCH = 200;           % candidate pool size per iteration
DT = runtime*60*60 / N_SIM;      % time per simulation step

% Toggle for candidate selection optimization:
%   true: choose candidate that minimizes effective acceleration error;
%   false: choose randomly among candidates that meet the angular constraint.
useOptimization = true; 

%% TRAJECTORY GENERATION (Run Once)
if exist('trajectory.mat','file')
    load('trajectory.mat','trajectory','candidateOrientations','r');
else
    % Generate random candidate points on the unit sphere.
    r = randn(3, LENGTH_R);
    r = r ./ vecnorm(r);
    
    % For context, store candidate coordinates.
    x_all = r(1,:);
    y_all = r(2,:);
    z_all = r(3,:);
    
    % Initialize the trajectory.
    first_point = [0; 1; 0];  % starting point on the unit sphere.
    distance = sqrt((x_all - first_point(1)).^2 + (y_all - first_point(2)).^2 + (z_all - first_point(3)).^2);
    [~, closest_indices] = mink(distance, 11);
    closest_indices = closest_indices(2:end);  % skip the first candidate.
    random_index = randi(length(closest_indices));
    second_point = r(:, closest_indices(random_index));
    first_vector = second_point - first_point;
    first_vector = first_vector / norm(first_vector);
    second_point = first_point + DS * first_vector;
    second_point = second_point / norm(second_point);
    
    trajectory = zeros(3, N_SIM+2);
    trajectory(:,1) = first_point;
    trajectory(:,2) = second_point;
    candidateOrientations = zeros(3, N_SIM+1);
    candidateOrientations(:,1) = first_vector;
    
    previous_point = second_point;
    last_vector = first_vector;
    
    % Precompute acceleration scaling factors.
    accel_vector_g = [0; -g; 0];
    if r_worst_case > 0
        a_centripetal_factor = (DS * r_worst_case / DT)^2 / r_worst_case;
        a_tangential_factor  = (DS * r_worst_case / DT)^2;
    else
        a_centripetal_factor = 0;
        a_tangential_factor  = 0;
    end
    
    % Simulation loop.
    for sim = 1:N_SIM
        vectors_to_candidates = r - previous_point;
        distances = vecnorm(vectors_to_candidates, 2, 1);
        [~, interim_indices] = mink(distances, NUM_DIST_SEARCH + 1);
        interim_indices = interim_indices(2:end);
        
        candidate_vectors = r(:, interim_indices);
        vecs_from_prev = candidate_vectors - previous_point;
        norms_vecs = vecnorm(vecs_from_prev, 2, 1);
        cos_angles = dot(vecs_from_prev, repmat(last_vector, 1, NUM_DIST_SEARCH), 1) ./ (norms_vecs * norm(last_vector));
        
        valid_mask = cos_angles >= MAX_COS_ANGLE;
        valid_indices = interim_indices(valid_mask);
        if isempty(valid_indices)
            disp('No valid candidate points found.');
            break;
        end
        
        if useOptimization
            numValid = length(valid_indices);
            effErrors = zeros(1, numValid);
            for k = 1:numValid
                cand_pt = r(:, valid_indices(k));
                candidate_direction = cand_pt - previous_point;
                candidate_direction = candidate_direction / norm(candidate_direction);
                temp_next = previous_point + DS * candidate_direction;
                temp_next = temp_next / norm(temp_next);
                a_cent = a_centripetal_factor * (-temp_next);
                a_tang = a_tangential_factor * candidate_direction;
                total_acc = accel_vector_g + a_cent + a_tang;
                effAccelCandidate = dot(temp_next, total_acc);
                effErrors(k) = abs(effAccelCandidate);
            end
            [~, bestIdx] = min(effErrors);
            chosen_index = valid_indices(bestIdx);
        else
            chosen_index = valid_indices(randi(length(valid_indices)));
        end
        
        chosen_candidate = r(:, chosen_index);
        step_direction = chosen_candidate - previous_point;
        step_direction = step_direction / norm(step_direction);
        next_point = previous_point + DS * step_direction;
        next_point = next_point / norm(next_point);
        
        trajectory(:, sim+2) = next_point;
        candidateOrientations(:, sim+1) = step_direction;
        
        last_vector = step_direction;
        previous_point = next_point;
    end
    save('trajectory.mat', 'trajectory', 'candidateOrientations', 'r');
end

%% FIGURE 1: Full Sphere with First 50 Trajectory Points and Cone Search Regions
figure;
% Draw a full unit sphere.
[sx, sy, sz] = sphere(50);
surf(sx, sy, sz, 'FaceAlpha', 0.1, 'EdgeColor', 'none');
colormap([0.8 0.8 0.8]);
hold on;
% Plot a subset of candidate points in blue with reduced opacity.
hScatter = scatter3(r(1,1:500:end), r(2,1:500:end), r(3,1:500:end), 5, 'b', 'filled');
if isprop(hScatter, 'MarkerFaceAlpha')
    hScatter.MarkerFaceAlpha = 0.3;
end

% Plot only the first 50 segments (i.e. first 51 points) of the trajectory.
numSegmentsToShow = 50;
if size(trajectory,2) < numSegmentsToShow+1
    numSegmentsToShow = size(trajectory,2) - 1;
end
for i = 1:numSegmentsToShow
    p1 = trajectory(:,i);
    p2 = trajectory(:,i+1);
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'r-', 'LineWidth', 2);
end

% For each of the first 50 segments, overlay the candidate region as a cone.
cone_length = DS;
half_angle_rad = deg2rad(MAX_ANGLE_DEG);
circle_radius = cone_length * tan(half_angle_rad);
for i = 1:numSegmentsToShow
    apex = trajectory(:, i);
    orient = candidateOrientations(:, i);
    basis = null(orient');
    if size(basis,2) >= 2
        u1 = basis(:,1);
        u2 = basis(:,2);
        theta_circle = linspace(0, 2*pi, 50);
        circle_pts = zeros(3, length(theta_circle));
        for j = 1:length(theta_circle)
            circle_pts(:, j) = apex + orient*cone_length + circle_radius*(cos(theta_circle(j))*u1 + sin(theta_circle(j))*u2);
        end
        plot3(circle_pts(1,:), circle_pts(2,:), circle_pts(3,:), 'm-', 'LineWidth', 1.5);
        for j = 1:10:length(theta_circle)
            plot3([apex(1) circle_pts(1,j)], [apex(2) circle_pts(2,j)], [apex(3) circle_pts(3,j)], 'm--');
        end
    end
end
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Full Sphere with First 50 Trajectory Points and Candidate Regions');
axis equal; grid on;
hold off;

%% FIGURE 2: Schematic of RPM Frames and Position Vector
% Use the 50th trajectory point (if available) as the target.
if size(trajectory,2) >= 51
    target_point = trajectory(:,51);
else
    target_point = trajectory(:,end);
end
% Convert target point to spherical coordinates.
[azimuth, elevation, ~] = cart2sph(target_point(1), target_point(2), target_point(3));

% Parameters for the frames.
outerFrameSide = 0.4;       % Outer square side length.
outerFrameThickness = 0.05; % Frame border thickness.
outerFrameDepth = 0.05;     % Extrusion depth.

innerFrameSide = 0.3;       % Inner square side length.
innerFrameThickness = 0.05; % Frame border thickness.
innerFrameDepth = 0.05;     % Extrusion depth.

figure;
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Schematic of RPM Frames and Position Vector');

% Outer frame: rotate about the vertical (z) axis by the target azimuth.
R_outer = [cos(azimuth) -sin(azimuth) 0;
           sin(azimuth)  cos(azimuth) 0;
           0            0           1];
drawSquareFrame(zeros(3,1), outerFrameSide, outerFrameThickness, outerFrameDepth, R_outer, [0.5 0.5 0.5]);

% Inner frame: attached to outer frame and rotated about the x-axis by the target elevation.
R_inner = R_outer * [1 0 0;
                     0 cos(elevation) -sin(elevation);
                     0 sin(elevation)  cos(elevation)];
drawSquareFrame(zeros(3,1), innerFrameSide, innerFrameThickness, innerFrameDepth, R_inner, [0.5 0.5 0.5]);

% Draw a blue position vector arrow from the inner frame's center along its normal.
% The inner frame's normal is given by R_inner*[0;0;1].
n_inner = R_inner * [0;0;1];
arrowLength = 0.3;
quiver3(0,0,0, n_inner(1)*arrowLength, n_inner(2)*arrowLength, n_inner(3)*arrowLength, ...
    'LineWidth', 2, 'MaxHeadSize', 2, 'Color', 'b');
hold off;

%% FUNCTION: drawSquareFrame
function drawSquareFrame(center, L, t, d, R, color)
    % Draws a closed square frame by representing each side as a rectangular prism.
    % center: 3x1 vector specifying the frame center.
    % L: outer square side length.
    % t: frame border thickness.
    % d: extrusion depth (in z-direction).
    % R: 3x3 rotation matrix for frame orientation.
    % color: color specifier.
    
    % Top side: centered at (0, L/2 - t/2, 0), dimensions: [L, t, d].
    topCenter_local = [0; L/2 - t/2; 0];
    topDims = [L, t, d];
    % Bottom side: centered at (0, -(L/2 - t/2), 0).
    bottomCenter_local = [0; -(L/2 - t/2); 0];
    bottomDims = topDims;
    % Left side: centered at (-(L/2 - t/2), 0, 0), dimensions: [t, L - 2*t, d].
    leftCenter_local = [-(L/2 - t/2); 0; 0];
    leftDims = [t, L - 2*t, d];
    % Right side: centered at (L/2 - t/2, 0, 0).
    rightCenter_local = [L/2 - t/2; 0; 0];
    rightDims = leftDims;
    
    % Compute global centers.
    topCenter = R * topCenter_local + center;
    bottomCenter = R * bottomCenter_local + center;
    leftCenter = R * leftCenter_local + center;
    rightCenter = R * rightCenter_local + center;
    
    % Draw the four rectangular prisms.
    drawRectangularPrism(topCenter, topDims, R, color);
    drawRectangularPrism(bottomCenter, bottomDims, R, color);
    drawRectangularPrism(leftCenter, leftDims, R, color);
    drawRectangularPrism(rightCenter, rightDims, R, color);
end

%% FUNCTION: drawRectangularPrism
function drawRectangularPrism(center, dims, transform, color)
    % Draws a rectangular prism centered at 'center' with dimensions dims = [w, L, d],
    % oriented by the 3x3 matrix 'transform', with a specified color.
    w = dims(1)/2;
    L = dims(2)/2;
    d = dims(3)/2;
    vertices = [ -w, -L, -d;
                  w, -L, -d;
                  w,  L, -d;
                 -w,  L, -d;
                 -w, -L,  d;
                  w, -L,  d;
                  w,  L,  d;
                 -w,  L,  d ];
    vertices = (transform * vertices')';
    vertices = vertices + center';
    
    faces = [1 2 3 4;
             5 6 7 8;
             1 2 6 5;
             2 3 7 6;
             3 4 8 7;
             4 1 5 8];
    
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', color, 'FaceAlpha', 0.5, 'EdgeColor', 'k', 'LineWidth', 1.5);
end
