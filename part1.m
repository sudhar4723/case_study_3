rays_in = [10 0 10 0 10; pi/20 pi/20 pi/20 pi/20 pi/20; 0 0 0 0 0; 0 0 0 0 0];
M = [1 3 0 0; 0 1 0 0; 0 0 1 3; 0 0 0 1];
rays_out = M*rays_in;
ray_z = [zeros(1,size(rays_in,2)); 3*ones(1,size(rays_in,2))];
figure;
plot(ray_z, [rays_in(1,:); rays_out(1,:)]);



rays=load('lightField.mat');
%rays


%rays2img(rays_x,rays_y,5,1200)

%M = [1 3 0 0; 0 1 0 0; 0 0 1 3; 0 0 0 1];

%propagation = M*rays.rays;
rays_x = propagation(1,1:end);
%fns=fieldnames(rays);
%rays_x
rays_y = propagation(3, 1:end);
rays2img(rays_x,rays_y,5,1200)
propagation = M*rays.rays;
ray_z = [zeros(1,size(rays.rays,2)); 3*ones(1,size(rays.rays,2))];
figure;
plot(ray_z, [rays.rays(1,:); propagation(1,:)]);