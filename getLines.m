%% Get lines from PIC and Matlab Accumulator (Comparison)

%% Clear Workspace and Command Window | Close All Windows
clear all
clc
close all

%% Read Input Image
I  = load('inputImage.txt');

%% Get lines from Matlab Accumulator
[H_MATLAB,T,R] = hough(I);
drawLines(I,H_MATLAB,T,R,'Lines from Matlab Accumulator');

%% Get lines from PIC Accumulator
H_PIC = load('accumulator.txt');
drawLines(I,H_PIC,T,R,'Lines from PIC Accumulator');


%% FUNCTION: drawLines method
function drawLines(I,H,T,R,STR)
    %% Find Peaks
    P  = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));

    %% Find lines
    lines = houghlines(I,T,R,P,'FillGap',5,'MinLength',7);
    
    %% Plot Lines
    figure, imshow(I), hold on;
    title(STR);
    
    for k = 1:length(lines)
       xy = [lines(k).point1; lines(k).point2];
       plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
    end
    
    hold off;
end