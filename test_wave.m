clear all; clc; clf; close all;

Fs = 44000;
step_Size_1 = 51076057;
step_Size_2 = 64351799;

n = 10000;

int32_max = 2147483647;

freq_1 = (int32_max/step_Size_1);
freq_2 = (int32_max/step_Size_2);

%% Generate C sawtooth and square
y1 = zeros(n,1); %phase generator
y1_square = zeros(n,1);
y1_sine = zeros(n,1);
y1_triangle = zeros(n,1);
for i=1:size(y1,1)
    if (i == 1)
        y1(i,1) = 0;
    else
        temp = y1(i-1,1) + step_Size_1; 
        if temp > int32_max
            y1(i,1) = 0;
        else
            y1(i,1) = temp;
        end
    end
    if y1(i,1) > int32_max/2
        y1_square(i,1) = 0;
        y1_triangle(i,1) = -1 + (y1(i,1)/int32_max - 0.5)*4;
    else
        y1_square(i,1) = 1;
        y1_triangle(i,1) = 1 - y1(i,1)/int32_max*4;
    end
    y1_sine(i,1) = sin(pi*freq_1*i);
end
y_1_mean = mean(y1,1:Fs);
y1_AC = y1 - y_1_mean;

%% Generate E Sawtooth and square
y2 = zeros(n,1);
y2_square = zeros(n,1);
y2_sine = zeros(n,1);
y2_triangle = zeros(n,1);
for i=1:size(y2,1)
    y2_sine(i,1) = sin(pi*freq_2*i);
    if (i == 1)
        y2(i,1) = 0;
    else
        temp = y2(i-1,1) + step_Size_2; 
        if temp > int32_max
            y2(i,1) = 0;
        else
            y2(i,1) = temp;
        end
    end
    if y2(i,1) > int32_max/2
        y2_square(i,1) = 0;
        y2_triangle(i,1) = -1 + (y2(i,1)/int32_max - 0.5)*4;
    else
        y2_square(i,1) = 1;
        y2_triangle(i,1) = 1 - y2(i,1)/int32_max*4;
    end
end
y2_mean = mean(y2,1:Fs);
y2_AC = y2 - y2_mean;

%% Polyphony
y  = max(y1_AC,y2_AC);
y_square  = max(y1_square,y2_square);
y_triangle  = max(y1_triangle,y2_triangle);

y_sine = y1_sine + y2_sine;
% y_sine = zeros(10000,1);
% for i=1:size(y1_sine,1)
%     if y1_sine(i,1) > 0
%         y_sine(i, 1) = max(y1_sine(i,1), y2_sine(i,1));
%     else
%         y_sine(i, 1) = min(y1_sine(i,1), y2_sine(i,1));
%     end
% end

%%
figure(1); clf;
plot(y2/int32_max); hold on;
plot(y2_square); hold on;
plot(y2_triangle); hold on;
plot(y2_sine); hold on;
% plot(y_sine, 'linewidth',2);
legend("sawtooth","square","triangle","sine");
xlim([0 100000/1000])

%%
sound(y_sine, Fs);