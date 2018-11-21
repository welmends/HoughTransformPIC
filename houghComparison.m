%% Hough Transform - Matlab Code (Comparison between PIC and MATLAB implementation)

%% Clear Workspace and Command Window | Close All Windows
clear all
clc
close all

%% Read input image
input  = load('inputImage.txt');

%% Calculate the Hough Transform
[H_MATLAB,Theta,Rho] = hough(input);

%% Read PIC Accumulator
H_PIC = load('accumulator.txt');

%% Plot the Hough Accumulator of Matlab
figure(1)
imshow(rescale(H_MATLAB));
title('Matlab Accumulator');

%% Plot the Hough Accumulator of PIC
figure(2)
imshow(rescale(H_PIC));
title('PIC Accumulator');