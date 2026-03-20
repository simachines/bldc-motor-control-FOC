
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is part of the hoverboard-new-firmware-hack-FOC project
% Compared to previouse commutation method, this project implements
% FOC (Field Oriented Control) for BLDC motors with Hall sensors.
% The new control methods offers superior performanace
% compared to previous method featuring:
% >> reduced noise and vibrations
% >> smooth torque output
% >> improved motor efficiency -> lower energy consumption
%
% Author: Emanuel FERU
% Copyright © 2019-2021 Emanuel FERU <aerdronix@gmail.com>
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Clear workspace
close all
clear
clc

% Load model parameters
load BLDCmotorControl_data;
Ts                  = 5e-6;                         % [s] Model sampling time (200 kHz)
Ts_ctrl             = 6e-5;                         % [s] Controller sampling time (~16 kHz)
f_ctrl              = 16e3;                         % [Hz] Controller frequency = 1/Ts_ctrl (16 kHz)
% Ts_ctrl             = 12e-5;                      % [s] Controller sampling time (~8 kHz)

% Motor parameters
n_polePairs         = 6;                           % [-] Number of motor pole pairs
a_elecPeriod        = 360;                          % [deg] Electrical angle period
a_elecAngle         = 60;                           % [deg] Electrical angle between two Hall sensor changing events
a_mechAngle         = a_elecAngle / n_polePairs;    % [deg] Mechanical angle between two Hall sensor changing events
r_whl               = 6.5 * 2.54 * 1e-2 / 2;        % [m] Wheel radius. Diameter = 6.5 inch (1 inch = 2.54 cm): Speed[kph] = rpm*(pi/30)*r_whl*3.6
i_sca               = 50;                           % [-] [not tunable] Scalling factor A to int16 (50 = 1/0.02)
r_mot               = 0.5;                          % [r] Phase to Phase resistance
r_motn              = r_mot/2;                      % [r] Phase to Neutral resistance = Phase to phase / 2
v_bus               = 24;                           % [V] bus voltage used for normalization
f_bw                = 1000;                         % [Hz] target current-loop bandwidth
L_ph                = 0.2e-3;                         % [H] phase-to-neutral inductance 

% --- PI gains (parallel PI, normalized by Vbus) ---
Kp = (2*pi*f_bw*L_ph) / v_bus;
Ki = Kp * (r_motn / L_ph);                 % same as (2*pi*f_bw*r_motn)/v_bus

% Target multiplier for filter cutoff vs bandwidth
filt_mult = 3;                           % CFG_CURR_FILT_TARGET_MULT
w_warp    = 2*pi*f_bw*Ts_ctrl;
R         = filt_mult;

warp_mult = R + (R^2*w_warp)/2 + (R^3*w_warp^2)/6 + (R^4*w_warp^3)/24 + (R^5*w_warp^4)/120;

% Current filter coefficient (same formula as firmware)
cf_curr_filt = Ts_ctrl / (Ts_ctrl + (1 / (2*pi*(f_bw*warp_mult))));

% Motor Angle Measurement (e.g. using an encoder)
b_angleMeasEna      = 1;                % [-] Enable flag for external mechanical motor angle sensor: 0 = estimated (default), 1 = measured
a_cpr               = 65535;
a_fcpr              = 1/65535;

% Sine/Cosine wave look-up table
a_elecoffset        = a_elecPeriod/12;
a_mechoffset        = a_elecoffset/n_polePairs;
res_elecAngle       = 2;
a_elecAngle_XA      = 0:res_elecAngle:360;          % [deg] Electrical angle grid
r_sin_M1            = sin((a_elecAngle_XA + a_elecPeriod/12)*(pi/180));  % Note: 30 deg shift is to allign it with the Hall sensors position
r_cos_M1            = cos((a_elecAngle_XA + a_elecPeriod/12)*(pi/180));

% figure
% stairs(a_elecAngle_XA, r_sin_M1); hold on
% stairs(a_elecAngle_XA, r_cos_M1);
% legend('sin','cos');

%% Control selection
% Control type selection
CTRL_COM            = 0;                % [-] Commutation Control
CTRL_SIN            = 1;                % [-] Sinusoidal Control
CTRL_FOC            = 2;                % [-] Field Oriented Control (FOC)
z_ctrlTypSel        = CTRL_FOC;         % [-] Control Type Selection (default)

% Control model request
OPEN_MODE           = 0;                % [-] Open mode
VLT_MODE            = 1;                % [-] Voltage mode
SPD_MODE            = 2;                % [-] Speed mode
TRQ_MODE            = 3;                % [-] Torque mode
z_ctrlModReq        = TRQ_MODE;         % [-] Control Mode Request (default)

% Cruise control
b_cruiseCtrlEna     = 0;                % [-] Cruise control enable flag: 0 = disable (default), 1 = enable
n_cruiseMotTgt      = 0;                % [-] Cruise control motor speed target



% MCU selection
STM32F103           = 0;                % [-] Changes the maximum useable PWM resolution from 2000 to 3375
GD32F103            = 1;
mcu_model           = GD32F103;          

%% F01_Estimations
% Position Estimation Parameters
% Hall              = 4*hA + 2*hB + hC
% Hall              = [0 1 2 3 4 5 6 7]
vec_hallToPos       = [0 2 0 1 4 3 5 0];  % [-] Mapping Hall signal to position

% Speed Calculation Parameters
cf_speedCoef        = round(f_ctrl * a_mechAngle * (pi/180) * (30/pi));     % [-] Speed calculation coefficient (factors are due to conversions rpm <-> rad/s)
z_maxCntRst         = 2000;             % [-] Maximum counter value for reset (works also as time-out to detect standing still)
n_commDeacvHi       = 30;               % [rpm] Commutation method deactivation speed high
n_commAcvLo         = 15;               % [rpm] Commutation method activation speed low
dz_cntTrnsDetHi     = 40;               % [-] Counter gradient High for transient behavior detection (used for speed estimation)
dz_cntTrnsDetLo     = 20;               % [-] Counter gradient Low for steady state detection (used for speed estimation)
n_stdStillDet       = 3;                % [rpm] Speed threshold for Stand still detection
cf_currFilt         = 0.692;             % [%] Current filter coefficient [0, 1]. Lower values mean softer filter

%% F02_Diagnostics
%b_diagEna           = 0;                % [-] Diagnostics enable flag: 0 = Disabled, 1 = Enabled (default)
%t_errQual           = 0.24 * f_ctrl/3;  % [s] Error qualification time
%t_errDequal         = 1.80 * f_ctrl/3;  % [s] Error dequalification time
%r_errInpTgtThres    = 600;              % [-] Error input target threshold (for "Blocked motor" detection)

%% F03_Control_Mode_Manager
%dV_openRate         = 1000 / (f_ctrl/3);% [V/s] Rate for voltage cut-off in Open Mode (Sample Time included in the rate)
%% F02_Diagnostics
b_diagEna           = 0;                % [-] Diagnostics enable flag: 0 = Disabled, 1 = Enabled (default)
t_errQual           = 0.24 * f_ctrl;  % [s] Error qualification time
t_errDequal         = 1.80 * f_ctrl;  % [s] Error dequalification time
r_errInpTgtThres    = 600;              % [-] Error input target threshold (for "Blocked motor" detection)

%% F03_Control_Mode_Manager
dV_openRate         = 1000 / (f_ctrl);% [V/s] Rate for voltage cut-off in Open Mode (Sample Time included in the rate)

%% F04_Field_Weakening
b_fieldWeakEna      = 0;                % [-] Field weakening enable flag: 0 = disable (default), 1 = enable
r_fieldWeakHi       = 1000;             % [1000, 1500] Input target High threshold for reaching maximum Field Weakening / Phase Advance
r_fieldWeakLo       = 750;              % [ 500, 1000] Input target Low threshold for starting Field Weakening / Phase Advance
n_fieldWeakAuthHi   = 300;              % [rpm] Motor speed High for field weakening authorization
n_fieldWeakAuthLo   = 100;              % [rpm] Motor speed Low for field weakening authorization

% FOC method
id_fieldWeakMax     = 5 * i_sca;        % [A] Field weakening maximum current

% SIN method
a_phaAdvMax         = 25;               % [deg] Maximum phase advance angle


%% F05_Field_Oriented_Control
z_selPhaCurMeasABC  = 2;                % [-] Select measured current phases: {iA,iB} = 0; {iB,iC} = 1; {iA,iC} = 2

% Motor Limitations Calibratables
%cf_iqKiLimProt      = 60 / (f_ctrl/3);  % [-] Current limit protection integral gain (only used in VLT_MODE and SPD_MODE)
%cf_nKiLimProt       = 20 / (f_ctrl/3);  % [-] Speed limit protection integral gain (only used in VLT_MODE and TRQ_MODE)
%cf_KbLimProt        = 1000 / (f_ctrl/3);% [-] Back calculation gain for integral anti-windup
cf_iqKiLimProt      = 60 / (f_ctrl);  % [-] Current limit protection integral gain (only used in VLT_MODE and SPD_MODE)
cf_nKiLimProt       = 20 / (f_ctrl);  % [-] Speed limit protection integral gain (only used in VLT_MODE and TRQ_MODE)
%cf_KbLimProt         = 1000 / (f_ctrl);
cf_KbLimProt = Simulink.Parameter;
cf_KbLimProt.Value = 1000 / (f_ctrl);  % [-] Back calculation gain for integral anti-windup
cf_KbLimProt.DataType = 'fixdt(0, 16, 12)'; 
cf_KbLimProt.CoderInfo.StorageClass = 'ExportedGlobal';

% Voltage Limitations STM32
V_margin                 = 60;              % [-] Voltage margin to make sure that there is a sufficiently wide pulse for a good phase current measurement
STM32_Vd_max             = 1000;                                  
STM32_Vd_max_margin      = STM32_Vd_max - V_margin;
STM32_Vq_max_XA          = 0:30:STM32_Vd_max_margin;
STM32_Vq_max_M1          = sqrt(STM32_Vd_max_margin^2 - STM32_Vq_max_XA.^2);

% Voltage Limitations GD32
V_margin                 = 60;              % [-] Voltage margin to make sure that there is a sufficiently wide pulse for a good phase current measurement
GD32_Vd_max              = 1687; 
GD32_Vd_max_margin       = GD32_Vd_max - V_margin;
GD32_Vq_max_XA           = 0:20:GD32_Vd_max_margin;
GD32_Vq_max_M1           = sqrt(GD32_Vd_max_margin^2 - GD32_Vq_max_XA.^2);  % Circle limitations look-up table

% figure
% stairs(Vq_max_XA, Vq_max_M1); legend('V_{max}');

% Speed limitations
n_max               = 1950;             % [rpm] Maximum motor speed: [-1500, 1500]

% Current Limitations
i_max               = 10;               % [A] Maximum allowed motor current (continuous)
i_max               = i_max * i_sca;
iq_maxSca_XA        = 0:0.01:0.99;
iq_maxSca_XA        = fixpt_evenspace_cleanup(iq_maxSca_XA, ufix(16), 2^-16);   % Make sure the data is evely spaced up to the last bit
iq_maxSca_M1        = sqrt(1 - iq_maxSca_XA.^2);                                % Current circle limitations map
% figure
% stairs(iq_maxSca_XA, iq_maxSca_M1); legend('i_{maxSca}');
%-------------------------------

tgt_scale            = 2047;

% Q axis control gains
cf_iqKp             =  Kp;              % [-] P gain
%cf_iqKi             = 400 / (f_ctrl/3); % [-] I gain
cf_iqKi             = Ki / (f_ctrl); % [-] I gain

% D axis control gains
cf_idKp             =  Kp;              % [-] P gain
%cf_idKi             = 400 / (f_ctrl/3);  % [-] I gain
cf_idKi             = Ki / (f_ctrl);  % [-] I gain


% Speed control gains
cf_nKp              = 1.18;             % [-] P gain
%cf_nKi              = 20.4 / (f_ctrl/3);% [-] I gain
cf_nKi              = 20.4 / (f_ctrl);% [-] I gain

%-------------------------------
ff_gain              = ((r_motn / i_sca) * ((GD32_Vd_max_margin) / v_bus));

%% F06_Control_Type_Management

% Commutation method
z_commutMap_M1      = [-1 -1  0  1  1  0;   % Phase A
                        1  0 -1 -1  0  1;   % Phase B
                        0  1  1  0 -1 -1];  % Phase C  [-] Commutation method map                   

omega               = a_elecAngle_XA*(pi/180);
pha_adv             = 30;       % [deg] Phase advance to mach commands with the Hall position
r_sinPhaA_M1        = -sin(omega + pha_adv*(pi/180));
r_sinPhaB_M1        = -sin(omega - 120*(pi/180) + pha_adv*(pi/180));
r_sinPhaC_M1        = -sin(omega + 120*(pi/180) + pha_adv*(pi/180));

% Sinusoidal 3rd harmonic method
A                   = 1.15;     % Sine amplitude (tunable to get the Saddle sin maximum to value 1000)
sin3Arm             = -0.224*sin(3*(omega + pha_adv*(pi/180)));     % 3rd armonic
r_sin3PhaA_M1       = sin3Arm + A*r_sinPhaA_M1;
r_sin3PhaB_M1       = sin3Arm + A*r_sinPhaB_M1;
r_sin3PhaC_M1       = sin3Arm + A*r_sinPhaC_M1;

disp('---- BLDC_controller: Initialization OK ----');

%% Plot control methods
show_fig            = 0;

if show_fig
    
    % Apply scaling
    sca_factor          = 1000;     % [-] scalling factor (to avoid truncation approximations on integer data type)
    r_sinPhaA_M1sca     = sca_factor * r_sinPhaA_M1;
    r_sinPhaB_M1sca     = sca_factor * r_sinPhaB_M1;
    r_sinPhaC_M1sca     = sca_factor * r_sinPhaC_M1;
    r_sin3PhaA_M1sca    = sca_factor * r_sin3PhaA_M1;
    r_sin3PhaB_M1sca    = sca_factor * r_sin3PhaB_M1;
    r_sin3PhaC_M1sca    = sca_factor * r_sin3PhaC_M1;
    
    % Commutation method
    a_commElecAngle_XA  = [0 60 120 180 240 300 360];  % [deg] Electrical angle grid
    hall_A              = [0 0 0 1 1 1 1] + 4;
    hall_B              = [1 1 0 0 0 1 1] + 2;
    hall_C              = [0 1 1 1 0 0 0];    
    
    % SVM (Space Vector Modulation) calculation
    SVM_vec             = [r_sinPhaA_M1sca; r_sinPhaB_M1sca; r_sinPhaC_M1sca];
    SVM_min             = min(SVM_vec);
    SVM_max             = max(SVM_vec);
    SVM_sum             = SVM_min + SVM_max;
    SVM_vec             = SVM_vec - 0.5*SVM_sum;
    SVM_vec             = (2/sqrt(3))*SVM_vec;
        
    color = ['m' 'g' 'b'];
    lw = 1.5;
    figure
    s1 = subplot(231); hold on
    stairs(a_commElecAngle_XA, hall_A, color(1), 'Linewidth', lw);
    stairs(a_commElecAngle_XA, hall_B, color(2), 'Linewidth', lw);
    stairs(a_commElecAngle_XA, hall_C, color(3), 'Linewidth', lw);
    xticks(a_commElecAngle_XA);
    grid
    yticks(0:5);
    yticklabels({'0','1','0','1','0','1'});
    title('Hall sensors');
    legend('Phase A','Phase B','Phase C','Location','NorthEast');
    
    s2 = subplot(232); hold on
    stairs(a_commElecAngle_XA, hall_A, color(1), 'Linewidth', lw);
    stairs(a_commElecAngle_XA, hall_B, color(2), 'Linewidth', lw);
    stairs(a_commElecAngle_XA, hall_C, color(3), 'Linewidth', lw);
    xticks(a_commElecAngle_XA);
    grid
    yticks(0:5);
    yticklabels({'0','1','0','1','0','1'});
    title('Hall sensors');
    legend('Phase A','Phase B','Phase C','Location','NorthEast');
    
    s3 = subplot(233); hold on
    stairs(a_commElecAngle_XA, hall_A, color(1), 'Linewidth', lw);
    stairs(a_commElecAngle_XA, hall_B, color(2), 'Linewidth', lw);
    stairs(a_commElecAngle_XA, hall_C, color(3), 'Linewidth', lw);
    xticks(a_commElecAngle_XA);
    grid
    yticks(0:5);
    yticklabels({'0','1','0','1','0','1'});
    title('Hall sensors');
    legend('Phase A','Phase B','Phase C','Location','NorthEast');
    
    s4 = subplot(234); hold on
    stairs(a_commElecAngle_XA, sca_factor*[z_commutMap_M1(1,:) z_commutMap_M1(1,1)] + 6000, color(1), 'Linewidth', lw);
    stairs(a_commElecAngle_XA, sca_factor*[z_commutMap_M1(2,:) z_commutMap_M1(2,1)] + 3000, color(2), 'Linewidth', lw);
    stairs(a_commElecAngle_XA, sca_factor*[z_commutMap_M1(3,:) z_commutMap_M1(3,1)], color(3), 'Linewidth', lw);
    xticks(a_commElecAngle_XA);
    yticks(-1000:1000:7000);
    yticklabels({'-1000','0','1000','-1000','0','1000','-1000','0','1000'});
    ylim([-1000 7000]);
    grid
    title('Commutation method [0]');
    xlabel('Electrical angle [deg]');
    
    s5 = subplot(235); hold on
    plot(a_elecAngle_XA, r_sin3PhaA_M1sca, color(1), 'Linewidth', lw);
    plot(a_elecAngle_XA, r_sin3PhaB_M1sca, color(2), 'Linewidth', lw);
    plot(a_elecAngle_XA, r_sin3PhaC_M1sca, color(3), 'Linewidth', lw);
    xticks(a_commElecAngle_XA);
    ylim([-1000 1000])
    grid
    title('SIN method [1]');
    xlabel('Electrical angle [deg]');
    
    s6 = subplot(236); hold on
    plot(a_elecAngle_XA, SVM_vec(1,:), color(1), 'Linewidth', lw);
    plot(a_elecAngle_XA, SVM_vec(2,:), color(2), 'Linewidth', lw);
    plot(a_elecAngle_XA, SVM_vec(3,:), color(3), 'Linewidth', lw);
    xticks(a_commElecAngle_XA);
    ylim([-1000 1000])
    grid
    title('FOC method [2]');    
    xlabel('Electrical angle [deg]');
    
    linkaxes([s1 s2 s3 s4 s5 s6],'x');
    xlim([0 360]);
    
end

