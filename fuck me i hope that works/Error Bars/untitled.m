clear; clc;

d1 = importfile("weightedscoresHILLsaveFlipped.csv");
d2 = importfile("weightedscoresHILLsaveFlipped 2.csv");
d3 = importfile("weightedscoresHILLsaveFlipped 3.csv");
d4 = importfile("weightedscoresHILLsaveFlipped 4.csv");
d5 = importfile("weightedscoresHILLsaveFlipped 5.csv");

%% 
clc; 

max1 = max(d1)
max2 = max(d2)
max3 = max(d3)
max4 = max(d4)
max5 = max(d5)

%% 
plotdata = zeros(6000);

for item 