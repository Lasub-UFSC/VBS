clc;
clear all;
close all;
pkg load control;

% -----------------------------
% Definicao da Planta e do Controlador
% -----------------------------
s = tf('s');
G = -62.38 / (3*s^2 + 1.4*s); % Planta

% Ganhos do PID
reescaler = 0.75;
Kp = -0.00654 / reescaler;
Kd = -0.0237 / reescaler;
Ki = -0.000305 / (reescaler^(3/2));
Tf = 0.1;      % Constante de tempo do filtro derivativo

C_pid_real = Kp + Ki/s + (Kd*s)/(Tf*s + 1); % Controlador

% -----------------------------
% Simulacoes Iniciais
% -----------------------------
% Definindo o Sistema em malha fechada
T_pid = feedback(C_pid_real * G, 1);

% Vetor de tempo
t_final = 30;
t = 0:0.001:t_final;

% Funcao degrau atenuada por uma sigmoide
sigmoid = @(x, a) 1 ./ (1 + exp(-a * x));
u_s = sigmoid(t - 20, 0.5) * 10;

% Funcao rampa com saturacao em 1
u_r = max(0, min(t * 1, 10));

% Calculando a resposta da profundidade z(t)
[z_s, t_s_out] = lsim(T_pid, u_s, t);
[z_r, t_r_out] = lsim(T_pid, u_r, t);

% Calculando a posicao do pistao h(t)
T_h = C_pid_real / (1 + C_pid_real * G);
[h_s, t_h_s_out] = lsim(T_h, u_s, t);
[h_r, t_h_r_out] = lsim(T_h, u_r, t);

% Calculando a velocidade do sistema v(t)
v_s = diff(z_s) ./ diff(t_s_out);
v_r = diff(z_r) ./ diff(t_r_out);
t_V_out = t_s_out(1:end-1); % Ajusta o vetor de tempo

% Calculando a velocidade do pistao v_h(t)
v_h_s = diff(h_s) ./ diff(t_h_s_out);
v_h_r = diff(h_r) ./ diff(t_h_r_out);
t_v_out = t_h_s_out(1:end-1); % Ajusta o vetor de tempo

% -----------------------------
% Definicaoo dos Limites Fisicos
% -----------------------------
h_lim_extensao = 0.03;
h_lim_contracao = -0.02;
v_h_limite = 0.5814 / 1000;
v_limite = 0.2;

% -----------------------------
% Figura 1: Informacoes do Sistema
% -----------------------------
figure(1);

% Grafico 1: Resposta da Profundidade z(t) com entrada Sigmoide
subplot(2, 2, 1);
plot(t, z_s, 'b', 'LineWidth', 2); hold on;
plot(t, u_s, 'r--', 'LineWidth', 1.5); grid on;
title("Profundidade z(t) - Entrada Sigmoide");
xlabel("Tempo (s)"); ylabel("Profundidade z(t) [m]");
legend("Saída z(t)", "Referência", 'Location', 'southeast');

% Grafico 2: Resposta da Profundidade z(t) com entrada Rampa
subplot(2, 2, 2);
plot(t, z_r, 'b', 'LineWidth', 2); hold on;
plot(t, u_r, 'r--', 'LineWidth', 1.5); grid on;
title("Profundidade z(t) - Entrada Rampa");
xlabel("Tempo (s)"); ylabel("Profundidade z(t) [m]");
legend("Saída z(t)", "Referência", 'Location', 'southeast');

% Grafico 3: Velocidade do Sistema v(t) com entrada Sigmoide
subplot(2, 2, 3);
plot(t_V_out, v_s, 'm', 'LineWidth', 2); hold on;
plot(t_V_out, v_limite * ones(size(t_V_out)), 'k--', 'LineWidth', 1);
plot(t_V_out, -v_limite * ones(size(t_V_out)), 'k--', 'LineWidth', 1); grid on;
title("Velocidade do Sistema v(t) - Entrada Sigmoide");
xlabel("Tempo (s)"); ylabel("Velocidade v(t) [m/s]");
legend("Velocidade Calculada", "Velocidade de Linearização", 'Location', 'northeast');

% Grafico 4: Velocidade do Sistema v(t) com entrada Rampa
subplot(2, 2, 4);
plot(t_V_out, v_r, 'm', 'LineWidth', 2); hold on;
plot(t_V_out, v_limite * ones(size(t_V_out)), 'k--', 'LineWidth', 1);
plot(t_V_out, -v_limite * ones(size(t_V_out)), 'k--', 'LineWidth', 1); grid on;
title("Velocidade do Sistema v(t) - Entrada Rampa");
xlabel("Tempo (s)"); ylabel("Velocidade v(t) [m/s]");
legend("Velocidade Calculada", "Velocidade de Linearização", 'Location', 'northeast');


% -----------------------------
% Figura 2: Informacoes do Pistao
% -----------------------------
figure(2);

% Grafico 1: Posicao do Pistao h(t) com entrada Sigmoide
subplot(2, 2, 1);
plot(t_h_s_out, h_s, 'g', 'LineWidth', 2); hold on;
plot(t_h_s_out, h_lim_extensao * ones(size(t_h_s_out)), 'k--', 'LineWidth', 1);
plot(t_h_s_out, h_lim_contracao * ones(size(t_h_s_out)), 'k--', 'LineWidth', 1); grid on;
title("Posição do Pistão h(t) - Entrada Sigmoide");
xlabel("Tempo (s)"); ylabel("Deslocamento h(t) [m]");
legend("Posição Requerida h(t)", "Limites Físicos", 'Location', 'east');

% Grafico 2: Posicao do Pistao h(t) com entrada Rampa
subplot(2, 2, 2);
plot(t_h_r_out, h_r, 'g', 'LineWidth', 2); hold on;
plot(t_h_r_out, h_lim_extensao * ones(size(t_h_r_out)), 'k--', 'LineWidth', 1);
plot(t_h_r_out, h_lim_contracao * ones(size(t_h_r_out)), 'k--', 'LineWidth', 1); grid on;
title("Posição do Pistão h(t) - Entrada Rampa");
xlabel("Tempo (s)"); ylabel("Deslocamento h(t) [m]");
legend("Posição Requerida h(t)", "Limites Físicos", 'Location', 'east');

% Grafico 3: Velocidade do Pistao v_h(t) com entrada Sigmoide
subplot(2, 2, 3);
plot(t_v_out, v_h_s, 'c', 'LineWidth', 2); hold on;
plot(t_v_out, v_h_limite * ones(size(t_v_out)), 'k--', 'LineWidth', 1);
plot(t_v_out, -v_h_limite * ones(size(t_v_out)), 'k--', 'LineWidth', 1); grid on;
title("Velocidade do Pistão v_h(t) - Entrada Sigmoide");
xlabel("Tempo (s)"); ylabel("Velocidade v_h(t) [m/s]");
legend("Velocidade Requerida", "Limite Físico", 'Location', 'northeast');

% Grafico 4: Velocidade do Pistao v_h(t) com entrada Rampa
subplot(2, 2, 4);
plot(t_v_out, v_h_r, 'c', 'LineWidth', 2); hold on;
plot(t_v_out, v_h_limite * ones(size(t_v_out)), 'k--', 'LineWidth', 1);
plot(t_v_out, -v_h_limite * ones(size(t_v_out)), 'k--', 'LineWidth', 1); grid on;
title("Velocidade do Pistão v_h(t) - Entrada Rampa");
xlabel("Tempo (s)"); ylabel("Velocidade v_h(t) [m/s]");
legend("Velocidade Requerida", "Limite Físico", 'Location', 'northeast');

v_h_s_pico = max(abs(v_h_s));
v_h_r_pico = max(abs(v_h_r));
v_pico_s = max(abs(v_s));
v_pico_r = max(abs(v_r));

fprintf("Velocidade máxima permitida do pistão: %f m/s\n", v_h_limite);
fprintf('Pico de velocidade do pistão (sigmoide): %f m/s\n', v_h_s_pico);
fprintf('Pico de velocidade do pistão (rampa): %f m/s\n', v_h_r_pico);
fprintf('Pico de velocidade do sistema (sigmoide): %f m/s\n', v_pico_s);
fprintf('Pico de velocidade do sistema (rampa): %f m/s\n', v_pico_r);
