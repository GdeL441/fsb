% Sensor Achterkant
data = readmatrix('Logs/Sensor B.txt');% Data inlezen

% Tijdmatrix
sampling_interval = 0.05; % 0.05s tussen metingen
time = (0:length(data)-1) * sampling_interval; % X-as maken

% Grafiek voor de achterkant maken
figure;
plot(time, data); 
xlabel('Tijd (s)');
ylabel('GPIO-waarde (0-65535)');
title('Sensor Achterkant');
grid on;
hold off;

%-----------------------------------------------------------%

% Sensor Linkerkant
data = readmatrix('Logs/Sensor L.txt');% Data inlezen

% Tijdmatrix
sampling_interval = 0.05; % 0.05s tussen metingen
time = (0:length(data)-1) * sampling_interval; % X-as maken

% Grafiek voor de linkerkant maken
figure;
plot(time, data); 
xlabel('Tijd (s)');
ylabel('GPIO-waarde (0-65535)');
title('Sensor Links');
grid on;

%-----------------------------------------------------------%

% Sensor Rechterkant
data = readmatrix('Logs/Sensor R.txt');% Data inlezen

% Tijdmatrix
sampling_interval = 0.05; % 0.05s tussen metingen
time = (0:length(data)-1) * sampling_interval; % X-as maken

% Grafiek voor de rechterkant maken
figure;
plot(time, data); 
xlabel('Tijd (s)');
ylabel('GPIO-waarde (0-65535)');
title('Sensor Rechts');
grid on;
