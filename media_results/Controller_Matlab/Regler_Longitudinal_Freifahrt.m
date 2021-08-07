% Parameter des PID-Reglers
P = 0.0;
I = 0.3;
D = 0.0;
k = 0.3;

% Größen der zu regelnden Strecke
a = 10;
b = -4;

% Beobachtungsdauer
dt = 0.1;
n = 300;

% Variablen initialisieren
v_soll = 50 * 1/3.6*ones(1,n);
v_soll(1:100) = 0;
v_ist = zeros(1,n);
v_diff = zeros(1,n);
a_ist = zeros(1,n);
a_soll = zeros(1,n);
a_diff = zeros(1,n);
stellgroesse = zeros(1,n);
integrierer = 0;

% Simulation
for i = 2:(length(v_soll)-1)
    
    % Regeldifferenz
    v_diff(i) = v_soll(i) - v_ist(i);
    a_soll(i) = k * v_diff(i);
    a_diff(i) = a_soll(i) - a_ist(i);
    
    % PID-Regler
    integrierer = integrierer + I*a_diff(i)*dt;
    integrierer = max(min(integrierer,1),-1);
    stellgroesse(i) = D*((a_diff(i)-a_diff(i-1))/dt) + P*a_diff(i) + integrierer;
    stellgroesse(i) = min(max(stellgroesse(i),-1),1);
    
    % Zu regelnde Strecke
    a_ist(i+1) = a*stellgroesse(i) + b;
    v_ist(i+1) = v_ist(i) + dt*a_ist(i+1);
end

% Ergebnisse anzeigen
plot((0:(length(v_soll)-1))*dt,[v_soll' v_ist' a_soll' a_ist' stellgroesse']);
xlabel('Zeit in s');ylabel('Geschwindigkeit in m/s und Beschleunigung in m/s^2');
legend('Soll-Geschwindigkeit','Ist-Geschwindigkeit','Soll-Beschleunigung','Ist-Beschleunigung','Gas');
