% Parameter des PID-Reglers
P = 0.0;
I = 0.1;
D = 0.0;
k1 = -0.3;
k2 = 3.0;

% Größen der zu regelnden Strecke
a = 30;
b = 0;
v_Vorausfahrender = 120 * 1/3.6;

% Beobachtungsdauer
dt = 0.1;
n = 200;

% Variablen initialisieren
dist_soll = v_Vorausfahrender * 3.6/2*ones(1,n) + 2;
dist_ist = 100*ones(1,n);
v_soll = zeros(1,n);
v_ist = zeros(1,n);
a_ist = zeros(1,n);
a_soll = zeros(1,n);
a_diff = zeros(1,n);
stellgroesse = zeros(1,n);
integrierer = 0;

% Simulation
for i = 2:(length(v_soll)-1)
    
    % Regeldifferenz
    v_soll(i) = k1 * (dist_soll(i) - dist_ist(i));
    a_soll(i) = k2 * (v_soll(i) - (v_ist(i)-v_Vorausfahrender));
    a_diff(i) = a_soll(i) - a_ist(i);
    
    % PID-Regler
    integrierer = integrierer + I*a_diff(i)*dt;
    integrierer = max(min(integrierer,1),-1);
    stellgroesse(i) = D*((a_diff(i)-a_diff(i-1))/dt) + P*a_diff(i) + integrierer;
    stellgroesse(i) = min(max(stellgroesse(i),-1),1);
    
    % Zu regelnde Strecke
    a_ist(i+1) = a*stellgroesse(i) + b;
    v_ist(i+1) = v_ist(i) + dt*a_ist(i+1);
    dist_ist(i+1) = dist_ist(i) - dt*(v_ist(i+1)-v_Vorausfahrender);
end

% Ergebnisse anzeigen
t = (0:(length(v_soll)-1))*dt;
subplot(2,1,1);
plot(t,[dist_soll' dist_ist' v_soll' (v_ist-v_Vorausfahrender)']);
xlabel('Zeit in s');ylabel('Geschwindigkeit in m/s');
legend('Soll-Distanz','Ist-Distanz','Soll-Geschwindigkeit','Ist-Geschwindigkeit');
subplot(2,1,2);
plot(t,[a_soll' a_ist' stellgroesse']);
xlabel('Zeit in s');ylabel('Beschleunigung in m/s^2');
legend('Soll-Beschleunigung','Ist-Beschleunigung','Gas');
