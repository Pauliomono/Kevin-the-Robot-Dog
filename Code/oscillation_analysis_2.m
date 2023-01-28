clear
clc
close all

data = load("frequency3.txt");


hold on
plot(data(:,3)/1000,data(:,1))
%plot(data(:,1)/1000,data(:,7),'--')
%plot(data(:,1)/1000,data(:,5) + 40)
%axis([12 13 80 92]);

%period = .1314 s
%amplitude = 1.1 degrees 