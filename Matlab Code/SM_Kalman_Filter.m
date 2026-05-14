%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Title: Simulation of Micro-MMC SM Kalman Filter                   %
% Authors: Michael M.C. Merlin, Kenneth Tanfa                       %
% Copyright 2025                                                    %
% Implementation: Standard Kalman Filter Algorithm                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Source: https://en.wikipedia.org/wiki/Kalman_filter


%% Simulation Système LTV Descriptor avec Perturbations Variables
% Système : E*x_dot = A(t)*x + Bd*ud(t)

clear; clc;

%% --- Paramètres système ---
    % Paramètres électriques
fsw = 100e3;                % [Hz] Fréquence de commutation
fc =  fsw/8;                % [Hz] Fréquence du controle commande
fac = 50;                   % [Hz] Fréquence de modulation
wac = 2*pi*fac;             % Pulsation
R1 = 0.1; R2 = 0.1;         % [ohm] Résistances
Cdc = 10e-3;                % [F] Capacité bus DC
Rdc = 10e3;                 % [ohm] Résistance bus DC
L1 = 33e-6; L2 = 33e-6;     % [H] Inductances
C1 = 6.6e-6; C2 = 6.6e-6;   % [F] Capacités de sortie

    % Systeme d'États: x = [Vdc IL1 IL2 Vout1 Vout2]'
nx = 5; nu = 0; nud = 2; ny = nx;    % Taille des vecteurs
E = diag([Cdc L1 L2 C1 C2]);  % Matrice descriptor

    % Index de Modulation
m_mag = 0.4;                        % Magnitude of modulation
m1 = @(t) 0.5+m_mag*sin(wac*t);     % Fonction de modulation 1
m2 = @(t) 0.5-m_mag*sin(wac*t);     % Fonction de modulation 2

    % Matrice A variant dans le temps
A = @(t) [-1/Rdc   -m1(t)  -m2(t)   0     0;
           m1(t)   -R1      0      -1     0;
           m2(t)    0      -R2      0    -1;
           0        1       0       0     0;
           0        0       1       0     0];
B = [];
C = eye(nx);  % Matrice d'observation 
D = [];

    % Matrice des perturbations
Bd = [ 0  0;
       0  0;
       0  0;
      -1  0;
       0 -1];
Dd = zeros(ny,nud);

    % Perturbations (charges en courant anti-phasées)
I_mag = 1;                          % [A] Amplitude du courant
Iout1 = @(t) I_mag*cos(wac*t);      % Courant de sortie 1
Iout2 = @(t) -I_mag*cos(wac*t);     % Courant de sortie 2 (anti-phase)
Ud = @(t) [Iout1(t); Iout2(t)];     % Vecteur des perturbations

%% --- Configuration de la simulation ---
    % Timing
Tsim = 1.5*1/fac;          % [s] Durée de simulation (2 périodes de fac = 50 Hz)
dt = 1/(fc*10);            % Pas de temps (10 points par période de contrôle)
dt_control = 1/fc;         % Pas de temps du contrôleur
t_span = 0:dt:Tsim;        % Vecteur temps
tc_span = floor(t_span*fc)/fc; % Temps échantillonnés du contrôleur

% CONDITIONS INITIALES
x0 = [12; 0.5; -0.5; 5; 5];     % [Vdc=5V, courants et tensions nuls]

%% =================================================================
%% ÉTAPE 1: DÉFINITION DES MATRICES DE COVARIANCE DU FILTRE KALMAN
%% =================================================================
% Les valeurs des matrices de bruit de processus et de bruits de mesures 
% sont des valeurs possibles . J'estime que la matriced des bruits a plus
% d'impact que la matrice de processus 

fprintf('\n=== ÉTAPE 1: Paramètres du Filtre de Kalman ===\n');

Cobs = [C, zeros(nx,nud)];

% Q: Matrice theorique de covariance du bruit de processus w ~ N(0,Q)
% Représente l'incertitude sur le modèle dynamique
% x = [Vdc IL1 IL2 Vout1 Vout2]'
Q = diag([10e-6, 100e-6, 100e-6, 10e-6, 10e-6, 500e-6, 500e-6]);  
Qw = 1.0*Q(1:nx,1:nx);     % Bruit actuelle simulé
fprintf('Q définie: bruit de processus\n');

% R: Matrice theorique de covariance du bruit de mesure v ~ N(0,R)  
% Représente l'incertitude des capteurs
% y = [Vdc IL1 IL2 Vout1 Vout2]'
R = diag([1e-3, 1000e-6, 1000e-6, 20e-3, 20e-3]);  
Rv = diag([5, 10, 10, 5, 5]) .* R;     % Bruit actuelle simulé
fprintf('R définie: bruit de mesure\n');

K_k2 = [    0.0860,   -0.0465,   -0.0459,    0.0183,    0.0177; ...
           -0.0009,    0.7882,    0.0009,    0.0107,   -0.0002; ...
           -0.0009,    0.0009,    0.7881,   -0.0002,    0.0107; ...
            0.0366,    1.0722,   -0.0200,    0.0874,    0.0087; ...
            0.0353,   -0.0194,    1.0729,    0.0087,    0.0868; ...
            0.0001,    0.6484,   -0.0010,    0.0337,    0.0001; ...
            0.0007,   -0.0011,    0.6482,    0.0002,    0.0338 ]*0;

K_k3 = [    0.0860,   -0.0465,   -0.0459,    0.0183,    0.0177; ...
           -0.0000,    0.7882,    0.0000,    0.0107,   -0.0000; ...
           -0.0000,    0.0000,    0.7881,   -0.0000,    0.0107; ...
            0.0366,    1.0722,   -0.0200,    0.0874,    0.0087; ...
            0.0353,   -0.0194,    1.0729,    0.0087,    0.0868; ...
            0.0000,    0.6484,   -0.0010,    0.0337,    0.0000; ...
            0.0000,   -0.0000,    0.6482,    0.0000,    0.0338 ]*0;


%% =================================================================
%% ÉTAPE 2: PRÉALLOCATION DES VARIABLES DE SIMULATION
%% =================================================================

fprintf('\n=== ÉTAPE 2: Préallocation des variables ===\n');

Nt = length(t_span);
x_clean = zeros(nx,Nt);     % États système SANS bruit (référence parfaite)
x_noisy = zeros(nx,Nt);     % États système AVEC bruit
x_hat_save = zeros(nx+nud,Nt);  % États estimés par Kalman
x_hat_save2 = zeros(nx+nud,Nt); % États estimés par matrice moyenne Kalman
x_hat_save3 = zeros(nx+nud,Nt); % États estimés par matrice simplifiée
y_meas = zeros(ny,Nt);      % Mesures bruitées
P_save = zeros(nx+nud,nx+nud,Nt);   % Sauvegarde des matrices P

S_k = zeros(nx,nx);
s_k_range = zeros(nx,nx,Nt);
K_k = zeros(nx+nud,nx);
k_k_range = zeros(nx+nud,nx,Nt);



fprintf('Simulation de %d ms à une resolution de %d us (%d points)\n', Tsim*1e3, dt*1e6, Nt);
fprintf('3 systèmes: Clean, Noisy, Kalman\n');

%% =================================================================  
%% ÉTAPE 3: BOUCLE DE SIMULATION AVEC ALGORITHME KALMAN STANDARD
%% =================================================================

fprintf('\n=== ÉTAPE 3: Simulation avec Observer ===\n');
fprintf('Algorithme Wikipedia: Predict -> Update\n');

% Initialisation
x_clean(:,1) = x0;          % Système parfait
x_noisy(:,1) = x0;          % Système bruité
y_meas(:,1) = C*x0;         % Première mesure

% État initial estimé et sa covariance d'erreur
x_hat  = [x0;zeros(nud,1)];                              % État estimé initial
x_hat2 = [x0;zeros(nud,1)];                              % État estimé 2 initial
x_hat3 = [x0;zeros(nud,1)];                              % État estimé 3 initial
P = diag([1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2]); % Covariance d'erreur initiale P0
P_save(:,:,1) = P;

wtbr = waitbar(0,'Simulation en cours');
tic;
for k = 1:Nt-1
    t_k = t_span(k);
    t_kp1 = t_span(k+1);
    tc_k = tc_span(k);
    tc_kp1 = tc_span(k+1);
    % ^^ Creation des temps 
    
    %% SIMULATION DES SYSTÈMES
    sys_ct = ss(E\A(tc_k), E\Bd, C, Dd);
    sys_dt = c2d(sys_ct, dt);
    
    % 1. SYSTÈME SANS BRUIT (référence parfaite)
    x_clean(:,k+1) = sys_dt.A*x_clean(:,k) + sys_dt.B*Ud(t_k);
    
    % 2. SYSTÈME AVEC BRUIT (réalité)
    w_k = chol(Qw) * randn(nx,1);  % Bruit processus ~ N(0,Q)
    x_noisy(:,k+1) = sys_dt.A*x_noisy(:,k) + sys_dt.B*Ud(t_k) + w_k;
    
    % Génération des mesures bruitées a partir du systeme bruité pour le Kalman
    v_k = chol(Rv) * randn(ny,1);  % Bruit mesure ~ N(0,R)
    y_k = C*x_clean(:,k+1) + v_k;       % Mesures = système sans bruit + bruit capteur
    y_meas(:,k+1) = y_k;
    
    %% ALGORITHME DE KALMAN (version wikipedia )
    % Le Kalman utilise les mesures bruitées pour estimer le vrai état
    
    if(tc_k ~= tc_kp1)  % Instant de contrôle/estimation
        % fprintf('Kalman update à k=%d, t=%.4f s\n', k, t_kp1);
        wtbr = waitbar(k/Nt);
        
        % Matrices du système discrétisé à la fréquence de contrôle
        Aobs = [E\A(tc_k), E\Bd; zeros(nud,nx+nud)];
        sys_ct_kf = ss(Aobs, [], Cobs, []);
        sys_dt_kf = c2d(sys_ct_kf, dt_control);
        
        F_k = sys_dt_kf.A;  % Matrice de transition d'état
        B_k = sys_dt_kf.B;  % Matrice d'entrée  
        H_k = Cobs;            % Matrice d'observation
        u_k = Ud(tc_k);     % Entrée de contrôle/perturbation
        
        %% ===== ÉTAPE PREDICT (Prédiction) =====
        % fprintf('  -> PREDICT step\n');
        
        % Prédiction de l'état: x_{k|k-1} = F_k * x_{k-1|k-1} + B_k * u_k
        x_predict = F_k * x_hat; % + B_k * u_k *0;
        
        % Prédiction de la covariance: P_{k|k-1} = F_k * P_{k-1} * F_k^T + Q
        P_predict = F_k * P * F_k' + Q;
        
        %% ===== ÉTAPE UPDATE (Correction) =====
        % fprintf('  -> UPDATE step\n');
        
        % Innovation (résidu): y_tilde = z_k - H_k * x_{k|k-1}
        y_innovation = y_k - H_k * x_predict;
        
        % Covariance de l'innovation: S_k = H_k * P_{k|k-1} * H_k^T + R
        S_k = H_k * P_predict * H_k' + R;

        % Gain de Kalman: K_k = P_{k|k-1} * H_k^T * S_k^{-1}
        K_k = P_predict * H_k' / S_k;  %truncated matrix: (eye(5).*S_k)
        
        % Correction de l'état: x_{k|k} = x_{k|k-1} + K_k * y_tilde
        x_hat = x_predict + K_k * y_innovation;
        
        % Correction de la covariance: P_{k|k} = (I - K_k * H_k) * P_{k|k-1}
        I_KH = eye(nx+nud) - K_k * H_k;
        P = I_KH * P_predict;


        %% ===== Simplified OBSERVER =====
        x_hat2 = F_k * x_hat2 + K_k2 * (y_k - H_k*x_hat2);

        % Simple DT SS - NOT WORKING
        Adt = eye(nx+nud) + Aobs/fc;
        x_hat3 = Adt * x_hat3  + K_k3 * (y_k - Cobs*x_hat3);
    else
        % % Pas d'estimation - simple propagation
        % sys_ct_kf = ss(E\A(tc_k), E\Bd, C, Dd);
        % sys_dt_kf = c2d(sys_ct_kf, dt);
        % 
        % % Propagation uniquement
        % x_hat = sys_dt_kf.A * x_hat + sys_dt_kf.B * Ud(t_k);
        % P = sys_dt_kf.A * P * sys_dt_kf.A' + Q * (dt/dt_control);

        % x_hat = x_hat_save(:,k);
        % P = P_save(:,:,k);
    end
    
    s_k_range(:,:,k) = S_k;
    k_k_range(:,:,k) = K_k;

    % Sauvegarde des résultats
    x_hat_save(:,k+1) = x_hat;
    P_save(:,:,k+1) = P;
    x_hat_save2(:,k+1) = x_hat2;
    x_hat_save3(:,k+1) = x_hat3;
end
time_simulation = toc;
close(wtbr);
fprintf('\nSimulation terminée en %.4f s\n', time_simulation);

% figure(10), plot(reshape(k_k_range,25,Nt)')

%% =================================================================
%% ÉTAPE 4:  VISUALISATION
%% =================================================================

fprintf('\n=== ÉTAPE 4: Analyse des résultats ===\n');

% Calcul des signaux de référence pour le plot
m1_signal = arrayfun(m1, t_span);    % Signal de modulation m1
m2_signal = arrayfun(m2, t_span);    % Signal de modulation m2
Iout1_signal = arrayfun(Iout1, t_span);  % Courant de perturbation 1
Iout2_signal = arrayfun(Iout2, t_span);  % Courant de perturbation 2


%% VISUALISATION COMPLÈTE
fprintf('Création des graphiques...\n');
fprintf('Création de la figure 1 (simulation complete)...\n');

figure(1); clf;
% set(gcf, 'Position', [100, 100, 1400, 900]);

% Légendes
legend_text = {'Etat réel', ...
               'Estimation Kalman', 'Estimation Simple', 'Estimation Simplisme', ...
               'Mesures bruitées'};
YLIM_I = [-2, 2];
YLIM_V = [-1, 15];

% === Courants ===
subplot(2,3,1);
plot(t_span*1e3, x_clean(2,:), 'k-', 'LineWidth', 2);      % Référence parfaite
hold on;
% plot(t_span*1e3, x_noisy(2,:), 'g:', 'LineWidth', 1.5);    % Système bruité
plot(t_span*1e3, x_hat_save(2,:), 'r:', 'LineWidth', 1.5); % Estimation Kalman
plot(t_span*1e3, x_hat_save2(2,:), 'm:', 'LineWidth', 1.5); % Estimation Simple
plot(t_span*1e3, x_hat_save3(2,:), 'y:', 'LineWidth', 1.5); % Estimation Simpliscime
plot(t_span*1e3, y_meas(2,:), 'b.', 'MarkerSize', 2);      % Mesures bruitées
xlabel('t [ms]'); ylabel('I [A]'); 
title('Courant IL1');
ylim(YLIM_I); 
legend(legend_text, 'Location', 'best');
grid on; grid minor;

subplot(2,3,2);
plot(t_span*1e3, x_clean(3,:), 'k-', 'LineWidth', 2);
hold on;
% plot(t_span*1e3, x_noisy(3,:), 'g:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save(3,:), 'r:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save2(3,:), 'm:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save3(3,:), 'y:', 'LineWidth', 1.5);
plot(t_span*1e3, y_meas(3,:), 'b.', 'MarkerSize', 2);
xlabel('t [ms]'); ylabel('I [A]'); 
title('Courant IL2');
ylim(YLIM_I);
legend(legend_text, 'Location', 'best');
grid on; grid minor;

% === Tensions ===
subplot(2,3,3);
plot(t_span*1e3, x_clean(1,:), 'k-', 'LineWidth', 2);
hold on;
% plot(t_span*1e3, x_noisy(1,:), 'g:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save(1,:), 'r:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save2(1,:), 'm:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save3(1,:), 'y:', 'LineWidth', 1.5);
plot(t_span*1e3, y_meas(1,:), 'b.', 'MarkerSize', 2);
xlabel('t [ms]'); ylabel('V [V]'); 
title('Tension DC bus');
ylim(YLIM_V);
legend(legend_text, 'Location', 'best');
grid on; grid minor;

subplot(2,3,4);
plot(t_span*1e3, x_clean(4,:), 'k-', 'LineWidth', 2);
hold on;
% plot(t_span*1e3, x_noisy(4,:), 'g:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save(4,:), 'r:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save2(4,:), 'm:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save3(4,:), 'y:', 'LineWidth', 1.5);
plot(t_span*1e3, y_meas(4,:), 'b.', 'MarkerSize', 2);
xlabel('t [ms]'); ylabel('V [V]'); 
title('Tension Vout1');
ylim(YLIM_V);
legend(legend_text, 'Location', 'best');
grid on; grid minor;

subplot(2,3,5);
plot(t_span*1e3, x_clean(5,:), 'k-', 'LineWidth', 2);
hold on;
% plot(t_span*1e3, x_noisy(5,:), 'g:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save(5,:), 'r:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save2(5,:), 'm:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save3(5,:), 'y:', 'LineWidth', 1.5);
plot(t_span*1e3, y_meas(5,:), 'b.', 'MarkerSize', 2);
xlabel('t [ms]'); ylabel('V [V]'); 
title('Tension Vout2');
ylim(YLIM_V);
legend(legend_text, 'Location', 'best');
grid on; grid minor;


% === Signaux de modulations et courants de perturbations 
subplot(4,3,9);

% Signaux de modulation ===
plot(t_span*1e3, m1_signal, 'b-', 'LineWidth', 2);
hold on;
plot(t_span*1e3, m2_signal, 'r-', 'LineWidth', 2);
ylabel('Indices de modulation [-]');
ylim([0, 1]);

xlabel('t [ms]'); 
title('Signaux de modulation');
legend({'m1(t)', 'm2(t)'}, 'Location', 'best');
grid on; grid minor;


% === Signaux de modulations et courants de perturbations 
subplot(4,3,12);

% Signaux de perturbations ===
plot(t_span*1e3, Iout1_signal, 'k-', 'LineWidth', 1.5);
hold on;
plot(t_span*1e3, Iout2_signal, 'k:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save(6,:), 'r-', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save(7,:), 'r:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save2(6,:), 'm-', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save2(7,:), 'm:', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save3(6,:), 'y-', 'LineWidth', 1.5);
plot(t_span*1e3, x_hat_save3(7,:), 'y:', 'LineWidth', 1.5);
ylabel('Courants de charge [A]');
ylim([-1.5, 1.5]);

xlabel('t [ms]'); 
title('Disturbance Currents');
legend({'Iout1(t)', 'Iout2(t)', ...
        'Iout1_1(t)', 'Iout2_1(t)', ...
        'Iout1_2(t)', 'Iout2_2(t)', ...
        'Iout1_3(t)', 'Iout2_3(t)'}, ...
        'Location', 'best');
grid on; grid minor;

%% =================================================================
%% FIGURE 2: ZOOM SUR L'INTERVALLE 0-1 ms
%% ================================================================

if (false)
    fprintf('Création de la figure 2 (zoom 0-1 ms)...\n');
    
    % Sélection de l'intervalle 0-1 ms
    t_zoom_max = 1e-3;  % 1 ms
    idx_zoom = t_span <= t_zoom_max;
    t_zoom = t_span(idx_zoom);
    
    figure(2); clf;
    % set(gcf, 'Position', [150, 150, 1400, 900]);
    
    % === Courants (zoom) ===
    subplot(2,3,1);
    plot(t_zoom*1e3, x_clean(2,idx_zoom), 'k-', 'LineWidth', 2);
    hold on;
    plot(t_zoom*1e3, x_noisy(2,idx_zoom), 'g:', 'LineWidth', 1.5);
    plot(t_zoom*1e3, x_hat_save(2,idx_zoom), 'r--', 'LineWidth', 1.5);
    plot(t_zoom*1e3, y_meas(2,idx_zoom), 'b.', 'MarkerSize', 3);
    xlabel('t [ms]'); ylabel('I [A]'); 
    title('Courant IL1 (zoom 0-1 ms)');
    ylim(YLIM_I); 
    legend(legend_text, 'Location', 'best');
    grid on; grid minor;
    
    subplot(2,3,2);
    plot(t_zoom*1e3, x_clean(3,idx_zoom), 'k-', 'LineWidth', 2);
    hold on;
    plot(t_zoom*1e3, x_noisy(3,idx_zoom), 'g:', 'LineWidth', 1.5);
    plot(t_zoom*1e3, x_hat_save(3,idx_zoom), 'r--', 'LineWidth', 1.5);
    plot(t_zoom*1e3, y_meas(3,idx_zoom), 'b.', 'MarkerSize', 3);
    xlabel('t [ms]'); ylabel('I [A]'); 
    title('Courant IL2 (zoom 0-1 ms)');
    ylim(YLIM_I);
    legend(legend_text, 'Location', 'best');
    grid on; grid minor;
    
    % === Tensions (zoom) ===
    subplot(2,3,3);
    plot(t_zoom*1e3, x_clean(1,idx_zoom), 'k-', 'LineWidth', 2);
    hold on;
    plot(t_zoom*1e3, x_noisy(1,idx_zoom), 'g:', 'LineWidth', 1.5);
    plot(t_zoom*1e3, x_hat_save(1,idx_zoom), 'r--', 'LineWidth', 1.5);
    plot(t_zoom*1e3, y_meas(1,idx_zoom), 'b.', 'MarkerSize', 3);
    xlabel('t [ms]'); ylabel('V [V]'); 
    title('Tension DC bus (zoom 0-1 ms)');
    ylim(YLIM_V);
    legend(legend_text, 'Location', 'best');
    grid on; grid minor;
    
    subplot(2,3,4);
    plot(t_zoom*1e3, x_clean(4,idx_zoom), 'k-', 'LineWidth', 2);
    hold on;
    plot(t_zoom*1e3, x_noisy(4,idx_zoom), 'g:', 'LineWidth', 1.5);
    plot(t_zoom*1e3, x_hat_save(4,idx_zoom), 'r--', 'LineWidth', 1.5);
    plot(t_zoom*1e3, y_meas(4,idx_zoom), 'b.', 'MarkerSize', 3);
    xlabel('t [ms]'); ylabel('V [V]'); 
    title('Tension Vout1 (zoom 0-1 ms)');
    ylim(YLIM_V);
    legend(legend_text, 'Location', 'best');
    grid on; grid minor;
    
    subplot(2,3,5);
    plot(t_zoom*1e3, x_clean(5,idx_zoom), 'k-', 'LineWidth', 2);
    hold on;
    plot(t_zoom*1e3, x_noisy(5,idx_zoom), 'g:', 'LineWidth', 1.5);
    plot(t_zoom*1e3, x_hat_save(5,idx_zoom), 'r--', 'LineWidth', 1.5);
    plot(t_zoom*1e3, y_meas(5,idx_zoom), 'b.', 'MarkerSize', 3);
    xlabel('t [ms]'); ylabel('V [V]'); 
    title('Tension Vout2 (zoom 0-1 ms)');
    ylim(YLIM_V);
    legend(legend_text, 'Location', 'best');
    grid on; grid minor;
    
    % === Signaux de modulation et perturbations (zoom) ===
    subplot(2,3,6);
    yyaxis left;  % Axe gauche pour les indices de modulation
    plot(t_zoom*1e3, m1_signal(idx_zoom), 'b-', 'LineWidth', 2);
    hold on;
    plot(t_zoom*1e3, m2_signal(idx_zoom), 'r-', 'LineWidth', 2);
    ylabel('Indices de modulation [-]');
    ylim([0, 1]);
    
    yyaxis right;  % Axe droit pour les courants de perturbation
    plot(t_zoom*1e3, Iout1_signal(idx_zoom), 'g--', 'LineWidth', 1.5);
    plot(t_zoom*1e3, Iout2_signal(idx_zoom), 'm--', 'LineWidth', 1.5);
    ylabel('Courants de charge [A]');
    ylim([-1.5, 1.5]);
    
    xlabel('t [ms]'); 
    title('Signaux de référence (zoom 0-1 ms)');
    legend({'m1(t)', 'm2(t)', 'Iout1(t)', 'Iout2(t)'}, 'Location', 'best');
    grid on; grid minor;
end

%% Affichage final des statistiques
fprintf('\n=== Résultats finaux ===\n');

fprintf(['States:  [V_{DC};    I_{L1};     I_{L2};     V_{out1};   V_{out2};   I_{out1};   I_{out1}\n']);

% fprintf('État final PARFAIT:  [%.3f V; %.3f A; %.3f A; %.3f V; %.3f V]\n', x_clean(:,end));
% fprintf('État final BRUITÉ:   [%.3f V; %.3f A; %.3f A; %.3f V; %.3f V]\n', x_noisy(:,end));
% fprintf('État final KALMAN:   [%.3f V; %.3f A; %.3f A; %.3f V; %.3f V]\n', x_hat_save(:,end));

% RMS of steady state error
n_bound = floor([0.2,1.0]*Nt);
n_range = n_bound(1):n_bound(2);
Ud_t = Ud(t_span);
ss_error_hat = sum(sqrt(([x_clean(:,n_range);Ud_t(:,n_range)]-x_hat_save(:,n_range)).^2),2)/length(n_range);
fprintf('Erreur:  [%.3f mV; %.3f mA; %.3f mA; %.3f mV; %.3f mV; %.3f mA; %.3f mA]\n', ss_error_hat*1e3');
fprintf('[RMS value in steady state]\n');

fprintf('\n=== SIMULATION TERMINÉE AVEC SUCCÈS ===\n');
































%% END OF FILE