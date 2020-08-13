#Tobias Ecker OE3TEC 2020
close all;

pkg load signal

f = [0, 0.0625, 0.0625, 1];
m = [0, 0, 0.0625, 1];
hp = fir2(11, f, m);
disp(hp);
freqz(hp);