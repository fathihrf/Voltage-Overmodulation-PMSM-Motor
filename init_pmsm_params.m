% =========================================================================
% Script Inisialisasi Parameter SPMSM dan PID Controller
% =========================================================================
% Jalankan script ini (Run) sebelum menjalankan diagram.slx

% --- 1. Parameter Fisik Plant Motor SPMSM (Contoh/Asumsi) ---
Rs   = 1.0;        % Resistansi stator per fasa [Ohm]
Ls   = 0.005;      % Induktansi stator (Ld = Lq = Ls) [Henry]
P    = 4;          % Jumlah pasangan kutub (pole pairs)
Phif = 0.1;        % Fluks magnet permanen [Weber]
B    = 0.01;       % Koefisien viskositas/gesekan [N.m.s/rad]
J    = 0.05;       % Momen inersia mekanis rotor [kg.m^2]

% --- 2. Perhitungan Parameter PID FOC (Pole-Zero Cancellation) ---
% Menentukan response time / time-constant yang diinginkan
tau_i = 0.001;     % Time constant untuk current loop (1 milidetik)
tau_w = 0.050;     % Time constant untuk speed loop (50 milidetik)

% A. Gain PID Loop Arus Sumbu D (d-axis)
Kp_id = Ls / tau_i;  
Ki_id = Rs / tau_i;  
Kd_id = 0;           % Tidak perlu D-term untuk redam noise arus

% B. Gain PID Loop Arus Sumbu Q (q-axis)
Kp_iq = Ls / tau_i;  
Ki_iq = Rs / tau_i;  
Kd_iq = 0;           % Tidak perlu D-term untuk redam noise arus

% C. Gain PID Loop Kecepatan (Outer Loop)
% Kompensasi kutub mekanis, dikali balikan perbandingan torsi-arus: T_e = (3/2)*P*Phif*i_q
gain_torsi = 2.0 / (3.0 * P * Phif);

Kp_w = (J / tau_w) * gain_torsi;
Ki_w = (B / tau_w) * gain_torsi;
Kd_w = 0;            % Tidak perlu D-term (atau gunakan nilai sangat kecil)

% --- 3. Referensi (Input Konstanta yang bisa dipakai di Simulink) ---
omega_ref_sim = 50;  % Referensi target Kecepatan [rad/s]
id_ref_sim    = 0;   % Target id untuk SPMSM (biarkan selalu 0)
T_L_sim       = 2.0; % Beban torsi dari luar (Load Torque) [N.m]

disp('Selesai! Parameter SPMSM dan PID Gain telah berhasil dimuat ke dalam Workspace MATLAB.');
disp('Silakan jalankan simulasi Simulink Anda.');
