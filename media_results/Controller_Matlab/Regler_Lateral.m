% Parameter des PID-Reglers
P = 10;
I = 1;
D = 0.0;
k = 1;

% Beobachtungsdauer
dt = 0.01;
n = 10000;

% Größen der zu regelnden Strecke (a=Übersetzung des Lenkwinkels, b=Kurve)
a = 0.1;
%b = zeros(1,n);
b = 0.008 * min(100*(1:n)/n,1);
%b = 0.01 * ones(1,n);
%b(1:100) = 0;

v = 100 / 3.6;
T = 0.5;

% Variablen initialisieren
y_soll = 0 * ones(1,n);
y_soll(1:100) = 0;
y_ist = zeros(1,n);
y_diff = zeros(1,n);
alpha_ist = zeros(1,n);
alpha_soll = zeros(1,n);
alpha_diff = zeros(1,n);
stellgroesse = zeros(1,n);
integrierer = 0;

% Simulation
for i = 2:(length(y_soll)-1)
    
    % Regeldifferenz
    y_diff(i) = y_soll(i) - y_ist(i);
    alpha_soll(i) = k/(v*T) * y_diff(i);
    alpha_diff(i) = alpha_soll(i) - alpha_ist(i);
    
    % PID-Regler
    integrierer = integrierer + I*alpha_diff(i)*dt;
    integrierer = max(min(integrierer,1),-1);
    stellgroesse(i) = D*((alpha_diff(i)-alpha_diff(i-1))/dt) + P*alpha_diff(i) + integrierer;
    stellgroesse(i) = min(max(stellgroesse(i),-1),1);
    
    % Zu regelnde Strecke
    alpha_ist(i+1) = alpha_ist(i+1) + dt*v*a*stellgroesse(i) + dt*v*b(i);
    y_ist(i+1) = y_ist(i) + dt*v*sin(alpha_ist(i+1));
end

% Ergebnisse anzeigen
t = (0:(length(y_soll)-1))*dt;
subplot(2,1,1);
plot(t,[y_soll' y_ist']);
xlabel('Zeit in s');ylabel('Position in m');
legend('Soll-Position','Ist-Position');
subplot(2,1,2);
plot(t,[alpha_soll' alpha_ist' stellgroesse']);
xlabel('Zeit in s');ylabel('Winkel in Rad');
legend('Soll-Winkel','Ist-Winkel','Lenkung');
