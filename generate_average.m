clear all; clc; clf; close all;

stepSizes = [51076057,54113197,57330935,60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346,91007187,96418756];
length = size(stepSizes,2);
%%

averages = zeros(1, length);
for j=1:length
    y = zeros(22000,1);
    for i=1:size(y,1)
        if (i == 1)
            y(i,1) = 0;
        else
            temp = y(i-1,1) + stepSizes(1,j); 
            if temp > 2147483647
                y(i,1) = 0;
            else
                y(i,1) = temp;
            end
        end
    end
    y_mean = mean(y,1:22000);
    
    averages(1,j) = y_mean;
end