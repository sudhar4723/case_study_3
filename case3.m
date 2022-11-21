clc; clear; close all;
%% PART 1.2

% Initalize constants
angles = linspace(-pi/20, pi/20,5);
d = 0.2;
M = matrix_prop(d);
% Rays_in at x = 0.01 m
rays_in = [
    0.01*ones(1,size(angles,2));
    angles;
    zeros(1,size(angles,2));
    zeros(1,size(angles,2));
   ];

% Rays_in  at x = 0 m
rays_in_B = [
    zeros(1,size(angles,2));
    angles;
    zeros(1,size(angles,2));
    zeros(1,size(angles,2));
   ];
% Plot the rays
figure;
plotRays(rays_in, d,'red')
plotRays(rays_in_B, d,'blue')
title('Propogation of light')
xlabel('z(m)')
ylabel('x(m)')
hold off;


%% Part 1.3

load('lightField.mat')

% Get X and Y rays
raysX = rays(1,:);
raysY = rays(3,:);

sens_width = 0.005;
pixels_count = 200;
rays2img(raysX,raysY,sens_width,pixels_count);

title(sprintf('Sensor width = %.3f m, pixels count = %.0f', sens_width, pixels_count))

%% 1.3 d)


% Propogate the rays by D
raysD = M * rays;

% Sensor constants
sens_width = 0.005;
pixels_count = 200;
% Get X and Y rays
raysX = raysD(1,:);
raysY = raysD(3,:);
rays2img(raysX,raysY,sens_width,pixels_count);

title(sprintf('Sensor width = %.3f m, pixels count = %.0f, distance = %.3f', sens_width, pixels_count, d), FontSize=9)


%% Part 2)

%% 2.2)


% Bending matrix 
f = 0.150;
Mf = matrix_bend(f);

% Propogation after bending
d2 = f*d/(d-f);
M2 = matrix_prop(d2);

% Plot the figure
figure
plotBend(rays_in_B,M,d,M2*Mf,d2,'red');
plotBend(rays_in,M, d,M2*Mf,d2,'blue');
hold off;

%% 2.3)

% Defining distances and focal point
d1 = -1;
f = 0.5;
d2 = f*d1/(d1-f);

% Matricies descbre how lights behave
M1 = matrix_prop(d1);
Mf = matrix_bend(f);
M2 = matrix_prop(d2);

% Bend by f that propogates by d2
processedRays =  M2*Mf*M1*rays;

raysX = processedRays(1,:);
raysY = processedRays(3,:);

rays2img(raysX,raysY,sens_width,pixels_count);

% Sensor constants
sens_width = 0.005;
pixels_count = 200;

title(sprintf('Sensor width = %.3f m, pixels count = %.0f, processed', sens_width, pixels_count), FontSize=9)






%% IMPLEMENTATION OF FUNCTIONS

% Function to plot rays that travel d(m)
% Rays_in should be a 4 x N matrix
% M is a 4 x 4 matrix
% color is the color of the rays
function plotRays(rays_in,d,color)
M = matrix_prop(d);
rays_out = M * rays_in;
ray_z = [zeros(1,size(rays_in,2)); d*ones(1,size(rays_in,2))];
plot(ray_z, [rays_in(1,:); rays_out(1,:)], 'color', color)
hold on;
end

% Function that plot rays that travel d(m) then get 
% bent at d. 

% M is a 4x4 matrix : describe how rays travel before getting bent
% distance d
% M2Mf is 4x4 matrix : describe how rays travel after getting bent
% d2 is the distance that the rays travel after getting bent
% color is the color of the rays

function plotBend(rays, M, d, M2Mf,d2, color)

rays_out = M*rays;
rays_bend = M2Mf*rays_out;
z_pos = [0 d d+d2];

plot(z_pos, [rays(1,:); rays_out(1,:); rays_bend(1,:) ],'color', color)
hold on;
end

% Generate a matrix that describes how lights travel d(m)
function [M] = matrix_prop(d)
M = [
1 d 0 0;
0 1 0 0;
0 0 1 d;
0 0 0 1;
];
end

% Generate a matrix that describes how lights bend
function [Mf] = matrix_bend(f)
Mf = [
1       0   0       0;
-1/f    1   0       0;
0       0   1       0;
0       0   -1/f    1;
];
end
