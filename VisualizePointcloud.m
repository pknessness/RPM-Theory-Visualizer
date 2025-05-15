%T = readmatrix('logs/log_12-05-2025_11_54_25.csv'); %my profile, 1.5hr
%T = readmatrix('logs/log_12-05-2025_12_34_11.csv'); %5MB profile, ?
%T = readmatrix('logs/log_12-05-2025_12_56_22.csv'); %Jon's profile, 3hr
%T = readmatrix('logs/log_12-05-2025_15_27_53.csv'); %Jon's profile, 1m

%T = readmatrix('logs/log_12-05-2025_17_17_07.csv'); % My profile, 1hr
%T = readmatrix('logs/log_12-05-2025_16_54_58.csv'); % Jon profile, 1hr

%T = readmatrix('logs/log_14-05-2025_02_21_16_V_JONMODIFIED_600s_0.733rads.csv'); % Modified jon 2hr
T = readmatrix('logs/log_14-05-2025_02_33_58_V_JONMODIFIED_600s_0.733rads.csv'); % Modified jon 0.25hr

P = 16000; %number here is how many points we are calculating on
ColorByIndex = 0; %1 for coloring based on what time these points were hit, as opposed to density
K = 0.5;%// threshold factor

n = numel(T(:,2))/P; 


iTheta = T(1:n:end,2);
oTheta = T(1:n:end,3);

sz = size(iTheta,1);         % Shorthand for size of data

[x, y, z] = sph2cart(iTheta, oTheta, 1);
D = [x(:), y(:), z(:)];

if ColorByIndex == 0
    %// calculation of color vector
    [n,m] = ndgrid(1:sz,1:sz);
    %// euclidian distance of each point to every other point
    X = zeros(sz, sz);
    for ii = 1:sz
        X(ii,:) = sum((repmat(D(ii,:), sz, 1) - D).^2, 2); % Get standard Euclidean distance 
        %X(ii) = nnz(dists < tol); % Count number of points within tolerance
        fprintf('\rProgress: %3d%%', round(100*ii/sz));
    end
    %X = arrayfun(@(a,b) sum( (D(a,:) - D(b,:)).^2 ), n, m);
    
    %// sort distances of points
    Y = sort(X,2);
    %// calculate average distance of the closest T% of all points
    Z = mean(Y(:,2:ceil(sz*K)),2);
else
    Z = linspace(0,1,P);
end


%// plot
scatter3(x,y,z,20,Z,'filled');
axis equal;
colormap(flipud(cool))
colorbar;