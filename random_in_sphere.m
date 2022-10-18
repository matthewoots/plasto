clc
clear all
close all

size = 50;
rand_size = 1000;
pi = 3.1415926535897;
v = (1:size) / size;
v_theta = v * 2*pi;
v_phi = v * pi;
radius = 1;
count = 0;
for i = 1:size
    for j = 1:size
        r = radius * 1;
        count = count + 1;
        sinTheta = sin(v_theta(i)); 
        cosTheta = cos(v_theta(i));
        sinPhi = sin(v_phi(j));
        cosPhi = cos(v_phi(j));
        perimeter(count,:) = [r * sinPhi * cosTheta, ...
            r * sinPhi * sinTheta, ...
            r * cosPhi];
    end
end

a = -pi + 0.707;

for i = 1:rand_size
    yaw = [cos(a), -sin(a), 0; ...
        sin(a), cos(a), 0; ...
        0, 0, 1];

    t_rand = max(min([rand 0.70]),0.30);
    p_rand = max(min([rand 0.70]),0.30);
    theta = t_rand * 2.0*pi;
    phi = p_rand * pi;
    r = radius * rand;
    sinTheta = sin(theta); 
    cosTheta = cos(theta);
    sinPhi = sin(phi);
    cosPhi = cos(phi);
    random(i,:) = (yaw * [r * sinPhi * cosTheta; ...
        r * sinPhi * sinTheta; ...
        r * cosPhi])';
end



figure(1)
hold on
plot3(perimeter(:,1), perimeter(:,2), perimeter(:,3), ...
    '.', 'MarkerSize', 3);
plot3(random(:,1), random(:,2), random(:,3), ...
    '.', 'MarkerSize', 5);
xlabel("x/m");
ylabel("y/m");
zlabel("z/m");
hold off
grid on