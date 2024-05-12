function [ output_args ] = visualizeNormal( data_dir )
%VISUALIZENORMAL Summary of this function goes here
%   Detailed explanation goes here
data = load(data_dir);
x = data(:,1);
y = data(:,2);
z = data(:,3);

subplot(2,2,1)
scatter3(x,y,z);
title('3d normal');
xlabel('x');
ylabel('y');
zlabel('z');

subplot(2,2,2)
scatter(x,y);
title('normal in plane XY');
xlabel('x');
ylabel('y');

subplot(2,2,3)
yaw = atan2(y,x);
n = hist(yaw,360);
bar(n);

end

