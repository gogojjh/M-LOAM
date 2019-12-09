close all; clc;
% clear;

%%
cloud = pcread("right.pcd");
vertical_angle = atan2(cloud.Location(:, 3), sqrt(cloud.Location(:,1).*cloud.Location(:,1) +  cloud.Location(:,2).*cloud.Location(:,2)));
vertical_angle_sort = sort(vertical_angle)/pi*180;
figure, plot(1:length(vertical_angle_sort), vertical_angle_sort);

%%
img = imread("test_rangeimage.png");
