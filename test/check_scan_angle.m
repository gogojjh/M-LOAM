close all; clc;
% clear;

%%
cloud = pcread("handheld/left.pcd");
vertical_angle = atan2(cloud.Location(:, 3), sqrt(cloud.Location(:,1).*cloud.Location(:,1) +  cloud.Location(:,2).*cloud.Location(:,2))) / pi * 180.0;
[vertical_angle_sort, index] = sort(vertical_angle);
% figure, plot(1:length(vertical_angle_sort), vertical_angle_sort);
hortical_angle = atan2(cloud.Location(index,1), cloud.Location(index,2)) / pi * 180.0;

%%
% img = imread("test_rangeimage.png");

%%
plot(1:1:400, hortical_angle(1:400))