#Tobias Ecker OE3TEC 2020
#Design Hilbert Filter for STM32 SSB

pkg load signal

#gives the number of taps in the returned filter
n = 14; #if 12 (or 8, 4) every uneven koeffitient will be 0
#gives frequency at the band edges
f = [0.1 0.9]; # if sum is 1, every second tab will be 0
#gives amplitude at the band edges
a = [1 1];
bb = remez(n, f, a, "hilbert", 200000);

disp(bb);
figure
freqz(bb,1);