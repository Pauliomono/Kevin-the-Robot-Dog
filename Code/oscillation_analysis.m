clear
clc
close all

data = load("frequency3.txt");

size = length(data);
t = linspace(0,size/250,238175)';
newdata = data(1,4);
j = 0;

scored = abs(sum(data(:,1) - data(:,7)))/size;

hold on
plot(data(:,1)/1000,data(:,2))
%plot(data(:,1)/1000,data(:,7),'--')
%plot(data(:,1)/1000,data(:,5) + 40)
axis([12 13 80 92]);

%period = .1314 s
%amplitude = 1.1 degrees 