%% Initialization
clc; clear all; close all;

% Global Variables
g = 9.81;               % gravity
r_worst_case = 6*0.0254; % worst case radius (m)
N_SIM = 1000;            % number of simulations
LENGTH_R = 10000000;       % number of random points
DIM_BOX_SEARCH = 0.06;  % search box dimension
MAX_ANGLE_DEG = 5;     % maximum deviation angle in degrees
MAX_COS_ANGLE = cosd(MAX_ANGLE_DEG);
DS = 0.1;              % constant distance to travel along trajectory  
NUM_DIST_SEARCH = 200;  % number of points initially checked for distance constraint each loop
DT = 10*60*60/LENGTH_R; % time (s) per step assuming 10 hour runtime

micro = 64;              % specify microstepping resolution

% Generate random unit vectors
r = randn(3, LENGTH_R);
r = r ./ vecnorm(r);

% Extract x, y, z coordinates from random vector
x = r(1,:);
y = r(2,:);
z = r(3,:);

%figure 1 - Vizualisation of distribution of points on a sphere
figure(1)
scatter3(x(1:500:end), y(1:500:end), z(1:500:end), 'bo', 'filled') %
grid off
axis equal
shading interp
view(3)
hold on;


% Select initial location defined by the randomly chosen point and plot
%first_point = r(:, randi(LENGTH_R));
first_point = [0; 1; 0];
plot3(first_point(1), first_point(2), first_point(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
plot3(first_point(1), first_point(2), first_point(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
%hold off;

%%
% V2: Distance calculation using logical indexing
% Preallocate the distance array with default value
distance = 5 * ones(1, LENGTH_R);

% Logical condition to check proximity within DIM_BOX_SEARCH
in_box = abs(x - first_point(1)) < DIM_BOX_SEARCH | ...
         abs(y - first_point(2)) < DIM_BOX_SEARCH | ...
         abs(z - first_point(3)) < DIM_BOX_SEARCH;

% Calculate distance only for points within the box region
distance(in_box) = sqrt((x(in_box) - first_point(1)).^2 + ...
                        (y(in_box) - first_point(2)).^2 + ...
                        (z(in_box) - first_point(3)).^2);

% Finding and vizualizing the 10 closest points
% non_zero_distances = distance > 0; % Logical indexing for non-zero distances
[closest_distances, closest_indices] = mink(distance, 11);
closest_distances = closest_distances(2:end);
closest_indices = closest_indices(2:end);

% Extract coordinates of the closest points
closest_points_x = x(closest_indices);
closest_points_y = y(closest_indices);
closest_points_z = z(closest_indices);

%% Second Point
% Generate random index from 1 to number of elements in array of closest
% point, and define the second point from this:

random_index = randi(length(closest_indices));
second_point = [closest_points_x(random_index); closest_points_y(random_index); closest_points_z(random_index)];

%% First Vector

first_vector = (second_point-first_point) / norm(second_point-first_point);
second_point = first_point + first_vector*DS;
second_point = second_point / norm(second_point);

%% Continuing Path
previous_point = second_point;
last_vector = first_vector;

%% Set up storage vectors
total_vector = zeros(3, N_SIM + 2);                                 % Pre-allocate position vector
total_vector(1:3,1:2) = [first_point, second_point];
motor_input = zeros(2, N_SIM + 2);                                  % Pre-allocate vector storing steps between points
accel_dot = zeros(1,N_SIM + 2);                                     % Pre-allocate vector storing accel. dot products
avg_accel_dot = zeros(1, N_SIM + 2);                                % Pre-allocate effective g vector

%% Calculate acceleration for first 2 points
accel_vector_g = [0;-g;0];                                          % Calculate accel from downward gravity
accel_centripetal = (DS*r_worst_case/DT)^2/r_worst_case;            % Determine magnitude of centripetal accel
accel_tangential = (DS*r_worst_case/DT)^2*(10.35/r_worst_case);     % Determine magnitude of MAX tangential accel

accel_vector_centripetal = accel_centripetal * (-1*second_point);   % Calculate total accel for second point
accel_vector_tangential = accel_tangential * first_vector;

total_accel = accel_vector_g;                                       % Calculate effective G for first point
accel_dot(1) = dot(first_point, total_accel);
avg_accel_dot(1) = accel_dot(1);

total_accel = accel_vector_tangential + accel_vector_centripetal + accel_vector_g;  % Calculate effective G for second point
accel_dot(2) = dot(second_point, total_accel);
avg_accel_dot(2) = (sum(accel_dot))/2;

%% Calculate stepper motor inputs for first 2 points
theta = atan2d(first_point(3),first_point(1));
if theta < 0
    theta = theta + 360;
end
psi = acosd(first_point(2));  

steps_inner = floor((theta/360)*micro*200);
steps_outer = floor((psi/360)*micro*200);
motor_input(:,1) = [steps_outer; steps_inner];

theta = atan2d(second_point(3),second_point(1));
if theta < 0
    theta = theta + 360;
end
psi = acosd(second_point(2));  

steps_inner = floor((theta/360)*micro*200);
steps_outer = floor((psi/360)*micro*200);
motor_input(:,2) = [steps_outer; steps_inner];

%% Simulation Loop

for sim = 1:N_SIM
    % Compute distances to all other points using vecnorm
    vectors_to_next = r - previous_point; 
    distances = vecnorm(vectors_to_next, 2, 1);

    % Find indices of NUM_DIST_SEARCH closest points to previous point,
    % minus the previous point itself
    [interim_distances, interim_distance_indices] = mink(distances, NUM_DIST_SEARCH + 1);
    interim_distances = interim_distances(2:end);
    interim_distance_indices = interim_distance_indices(2:end); 

    % plot first step of valid points based on distance criteria
    % scatter3(interim_valid_points(1,:), interim_valid_points(2,:), interim_valid_points(3,:), 'yo', 'filled');

    %Find points satisfying distance AND angle constraints
    cosine_angles = dot(vectors_to_next(:,interim_distance_indices), repmat(last_vector, 1, NUM_DIST_SEARCH), 1) ./ (interim_distances .* norm(last_vector));
    interim_valid_angle = cosine_angles >= MAX_COS_ANGLE;

    valid_indices_both = interim_distance_indices(interim_valid_angle);
    valid_points_both = r(:,valid_indices_both);

    % Break if no valid points are found
    if all(~interim_valid_angle)
        disp('No valid points found under angle and distance constraints.');
        break;
    end

    %Plot all valid candidates found based on both constraints
    % scatter3(valid_points_both(1,:), valid_points_both(2,:), valid_points_both(3,:), 'mo', 'filled')

    % Select a new point randomly from this set
    new_point_index = valid_indices_both(randi(length(valid_indices_both)));
    new_point = r(:, new_point_index);

    % Set up new vector to interim chosen point and normalize
    vector_to_new = new_point - previous_point;
    vector_to_new_norm = vector_to_new / norm(vector_to_new);

    % Find next point along trajectory and project back onto the sphere
    next_point = previous_point + vector_to_new_norm*DS;
    next_point = next_point / norm(next_point);

    total_vector(:,sim+2) = next_point;

    % Calculate acceleration to get to next_point
    accel_vector_centripetal = accel_centripetal * (-1*next_point);
    accel_vector_tangential = accel_tangential * vector_to_new_norm;

    total_accel = accel_vector_tangential + accel_vector_centripetal + accel_vector_g;
    accel_dot(sim+2) = dot(next_point, total_accel);
    avg_accel_dot(sim+2) = sum(accel_dot)/(sim+2); 

    % Calculate stepper motor input to get to next_point (full step)
    theta = atan2d(next_point(3),next_point(1));
    if theta < 0
        theta = theta + 360;
    end
    psi = acosd(next_point(2));  

    steps_inner = floor((theta/360)*micro*200);
    steps_outer = floor((psi/360)*micro*200);

    motor_input(:, sim+2) = [steps_outer; steps_inner];

    %Find vector to actual next point to be commanded to motor
    vector_to_next = next_point - previous_point;
    last_vector = vector_to_next;

    % Visualization
    % plot3([previous_point(1), next_point(1)], [previous_point(2), next_point(2)], [previous_point(3), next_point(3)], 'g-');  % Green line for trajectory
    % scatter3(next_point(1), next_point(2), next_point(3), 'go', 'filled');
    % drawnow;  % Update the plot

    if ~ishandle(gcf)  % Stop if the figure is closed
        break;
    end

    previous_point = next_point;

end


%%
writematrix(motor_input','MOTORINPUTS_64_10000_5DEG.csv')

clc
figure
% plot(1:sim+2,avg_accel_dot/g,'r-','LineWidth',2)
title('Effective Acceleration vs. Number of Points')
axis tight
xlabel('# of Points')
ylabel('Effective Acceleration(g''s)')

fprintf('Effective Gravity after %i points is %d (g''s)\n', N_SIM+2, avg_accel_dot(end)/g)