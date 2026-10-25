%% Filtro de Kalman 
% El objetivo de esta tarea es que entiendan los efectos e implicaciones 
% del filtro de Kalman. 


% Supongamos que tenemos un sistema masa resorte amortiguador dado 
% por la siguiente representacion en variables de estado 

% PD: Vayan Leyendo este archivo y con CTRL + ENTER van corriendo las
% secciones. Las secciones se saparan con doble % => (%%) 
% De todas formas pondré CORRER al final de una linea que recomiendo correr
% con CTRL + ENTER 


clear; clc; close all;  % CORRER 
%% Parametros =========
m = 0.1; % [kg] 
c = 0.6; % [Ns/m]
k = 5; % [N/m]

A = [0 1; -k/m -c/m]; 
B = [0 1/m]'; 
C = eye(2); 
D = zeros(2, 1); 

% Construimos nuestro sistema con las matrices (state-space = ss) 
Gs = ss(A, B, C, D);  
Gs.StateName = {'posicion', 'velocidad'}; 
Gs.InputName = {'fuerza'}; 

Gs.OutputName = Gs.StateName; % Mapeo unitario de estados a salida
% CORRER 

step(Gs)

%% Revisamos su respuesta 
step(Gs); % CORRER 
%% Dinámicas no modeladas 
% Existen siempre dinámicas no modeladas que pueden afectar el sistema. 
% Por ejemplo, si el sistema que genera la fuerza (que no lo modelamos) 
% Presenta una respuesta de primer orden, se tendría que incluir en el
% modelo. Por cuestiones practicas no lo hacemos, es imposible tener un
% modelo exacto de la respuesta real. Pero observemos como cambiaría la
% respuesta

Hs = tf([1.01],[0.9, 1]); % Dinámica de primer orden no modelada 

Gs_no_modelada = tf(Gs) * Hs; 

step(Gs, Gs_no_modelada); 

%% Incertidumbre de los parámetros 
% Supongamos que conocemos los parámetros de nuestro modelo (k, m , c) con 
% un 5% de precisión. Recuerden que ningun valor se conoce con exactitud
% completa. Miremos el efecto 


m_delta = m * 1.05; % [kg] 
c_delta = c  * 0.95; % [Ns/m]
k_delta = k * 1.05; % [N/m]

A_d = [0 1; -k_delta/m_delta -c_delta/m_delta]; 
B_d = [0 1/m_delta]'; 
C_d = eye(2); 
D_d = 0; 

Gs_incertidumbre = ss(A_d, B_d, C_d, D_d); 

step(Gs, Gs_incertidumbre); % CORRER 

%% Como hacemos un Filtro de Kalman ? 
% Entendiendo que tenemos diferencias reales con el modelo teórico, 
% Surge la necesidad de aplicar un filtro de Kalman, que corrija 
% Tanto Esas diferencias de los modelos como compensar el efecto ruido 
% Un observador tradicional, mezclar observacion y calculo, pero no lo 
% cierra del todo al no tener una métrica de confianza (Cosa que si tiene 
% el filtro de Kalman), y no encuentra una ganancia de correccion optima. 
% Como si lo hace Kalman :D 


% Vamos a definir ahora las matrices de confianza y los sensores 


% Digamos que tenemos 3 sensores. 
% - 1 x Sensor de posición 
% - 1 x Sensor de Velocidad 
% - 1 x Sensor de Aceleracion. (% SIEMPRE TOCA MEDIR LA ENTRADA )

% Confianzas por caracteristicas de mis sensores 
sigma_p1 = 0.01; % Desviacion de mi sensor de posicion 1 
sigma_v1 = 0.02; % """"""""" velocidad 1

% Matrices de confianza (Covarianzas) 
Q = diag([6.9, 7.9]); 
R = diag([sigma_p1^2 , sigma_v1^2]); 

% CORRER 

%% END ====== Continuamos en Simulink 
% Matrices discretas

% x_k+1 =  x_k + v_k * delta_T  + 1/2m (a) * Ts^2
% v_k+1 =  v_k + a * delta_T

% Pero recordemos que a = (F - kx - cv)

% Sutitumos y encontramos las matrices 

Fs = 1000; 
Ts = 1/Fs; 

F = [1 - k/(2*m)*Ts^2, Ts - c/(2*m)*Ts^2;
     -k/m*Ts,           1 - c/m*Ts];

G = [1/(2*m)*Ts^2;
     Ts/m];

H = eye(2);

D_k = zeros(size(H, 1), 1);







