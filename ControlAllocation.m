function [U] = ControlAllocation_(UTrimVec,FltCon,Veh)
%% File Information
% ControlAllocation - vehicle-specific control allocation (student template)
%
% PURPOSE:
%   Convert control-group commands into the nine effector commands for the
%   Hero lift-plus-cruise vehicle. Complete each TODO using the requirements
%   below and in Assignment 3. This function is an algebraic mixer.
%
% USE:
%   Save the completed file as ControlAllocation.m. Call it from a separate
%   Control_Test_Script.m using U = ControlAllocation(UTrimVec,FltCon,Veh).
%   Use MATLAB R2024a-compatible syntax. No additional toolbox is required.
%   Remove each TODO error statement when that task is implemented.
%
% INPUTS:
%   UTrimVec : 6- or 7-element control-group vector, ordered as follows:
%      1. ulat  : normalized lateral command [-1,+1]
%      2. ulon  : normalized longitudinal command [-1,+1]
%      3. udir  : normalized directional command [-1,+1]
%      4. MPRPM : common cruise-propeller speed [RPM]
%      5. LPRPM : common lift-propeller speed [RPM]
%      6. NAC   : common nacelle angle [deg], reserved in this vehicle
%      7. dflp  : optional direct flap command [deg], limited to [0,30]
%   FltCon.KEAS : equivalent airspeed [knots]
%   Veh.Aero.VTXN : transition airspeed [knots]
%   Veh.ControlAllocation.KB_roll  : differential RPM scale for ulat [RPM]
%   Veh.ControlAllocation.KB_pitch : differential RPM scale for ulon [RPM]
%   Veh.ControlAllocation.KB_yaw   : differential RPM scale for udir [RPM]
%   Veh.ControlAllocation.dBmat_LP : 4-by-4 lift-propeller mixing matrix;
%       rows = LP-1 through LP-4; columns = collective, roll, pitch, yaw.
%       In Control_Test_Script.m, fill the blank matrix entries with +1 or
%       -1 using the instructor-provided format, propeller numbering,
%       positions, and rotation directions. The instructor supplies KB values.
%
% OUTPUT:
%   U : 9-by-1 vector in this exact order:
%      1. Left aileron, outboard [deg, positive trailing-edge down]
%      2. Right aileron, outboard [deg, positive trailing-edge down]
%      3. Left ruddervator [deg, positive trailing-edge down]
%      4. Right ruddervator [deg, positive trailing-edge down]
%      5-8. LP-1, LP-2, LP-3, LP-4 speeds [RPM]
%      9. MP-1 cruise-propeller speed [RPM]
%
% SCOPE:
%   NAC and dflp are retained for interface compatibility. Neither produces
%   an output in this nine-effector version. Do not add extra U entries or
%   apply dflp to the ailerons. The provided flap schedule is retained.
%   The RPM mixer does not impose propeller-speed limits. Select test inputs
%   that keep all commanded speeds in the permitted vehicle range.
%
% ORIGINAL FUNCTION:
%   Anthony M. Comer, Oklahoma State University
%   anthony.comer@okstate.edu
%   Student name: Antonio Chappell
%
%% Hero Lift + Cruise Control Effectors

% 1. Left aileron, outboard (+TED)
% 2. Right aileron, outboard (+TED)
% 3. Left ruddervator (+TED)
% 4. Right ruddervator (+TED)
% 5. LP-1 Prop RPM
% 6. LP-2 Prop RPM
% 7. LP-3 Prop RPM
% 8. LP-4 Prop RPM
% 9. MP-1 Prop RPM

%% UTrimVec Defintion

% 1. ulat --> normalized lateral control command [-1,+1]
% 2. ulon --> normalized longitudinal control command [-1,+1]
% 3. udir --> normalized directional control command [-1,+1]
% 4. MPRPM --> common MPRPM component [0,MPRPMmax]
% 5. LPRPM --> common LPRPM component [0,LPRPMmax]
% 6. NAC --> common nacelle angle [0,90]
% 7. dflp --> direct flap command [0,+20]

%% Initializations / Settings (DO NOT EDIT)
U = zeros(9,1); % this corresponds to the number of outputs

% maximum control surface deflections [deg]
da_max = 30;
de_max = 30;
dr_max = 30;
df_max = 30;

%% Unpack Incoming Trim Vector (DO NOT EDIT)

% no need to edit below:
ulat = UTrimVec(1);
ulon = UTrimVec(2);
udir = UTrimVec(3);
MPRPM = UTrimVec(4);
LPRPM = UTrimVec(5);
NAC = UTrimVec(6);

if length(UTrimVec) > 6
    % sim mode
    dflp = UTrimVec(7);
else
    % trim mode
    dflp = -1;
end

% limit possible inputs
ulat = max(-1,min(ulat,1));
ulon = max(-1,min(ulon,1));
udir = max(-1,min(udir,1));

%% Flap Scheduling (DO NOT EDIT)

% x1 = start speed | x2 = end speed
% m = 1/(x2 - x1)
% wash-out = max(0,min(1 - m*(x - (x2-1/m)),1))
% wash-in = max(0,min(m*(x - (x2-1/m)),1))

% control system speed [kts]
V = FltCon.KEAS;
% transition speed [kts]
VTXN = Veh.Aero.VTXN;
% start/stop blending speeds (wash-out)
x1 = VTXN - 5;
x2 = VTXN + 0;
m = 1/(x2-x1);
z_txn = max(0,min(1 - m*(V - (x2-1/m)),1));

% start/stop blending speeds (wash-out)
x1 = VTXN + 0;
x2 = VTXN + 5;
m = 1/(x2-x1);
z_flap = max(0,min(1 - m*(V - (x2-1/m)),1));

% flap scheduling (during trim)
if dflp == -1
    dflp = 30*z_flap;
end
% limit flap command
dflp = min(df_max,max(0,dflp));

%% Control Effector Allocation

% left aileron (+TED)
da1 = [da_max * ulat];
% right aileron (+TED) 
da2 = [-da_max * ulat];

% left ruddervator (+TED/+TEL)
dr1 = [(de_max * -ulon - dr_max * udir)]; % Hint: it's affected by 2 inputs!
% right ruddervator (+TED/+TER)
dr2 = [(de_max * -ulon + dr_max * udir)]; % Hint: it's affected by 2 inputs!

% ensure they cannot saturate
dr1 = max(-de_max,min(dr1,de_max));
dr2 = max(-de_max,min(dr2,de_max));

%% Propulsor Allocation

% differential RPM limits
dN_lat = 1500; % +/- 1500 RPM at maximum roll input
dN_lon = 1500; % +/- 1500 RPM at maximum pitch input
dN_dir = 2000; % +/- 2000 RPM at maximum yaw input

% lift props wash-out linearly during forward flight mode
LP_roll = z_txn;
LP_yaw = z_txn;
LP_pitch = z_txn;

% compute differential RPM equivalents from incoming normalized commands
% and include the wash-in/wash-out terms
dlat = ulat*dN_lat*LP_roll;
dlon = ulon*dN_lon*LP_pitch;
ddir = udir*dN_dir*LP_yaw;

% common RPM differs in forward-flight as the props shut down
MPRPM0 = MPRPM;
LPRPM0 = LPRPM;

% propulsion mixing term
dBmat_LP = [...
    +[1],+[1],+[1],+[1]; % LP-1 (front-left) --> rotates CCW as viewed from above
    +[1],+[-1],+[1],+[-1]; % LP-2 (front-right) --> rotates CW as viewed from above
    +[1],+[1],+[-1],+[-1]; % LP-3 (back-left) --> rotates CW as viewed from above
    +[1],+[-1],+[-1],+[1]; % LP-4 (back-right) --> rotates CCW as viewed from above
    ];

% mixing matrix
N_LP = dBmat_LP*[LPRPM0;dlat;dlon;ddir];
N_MP = MPRPM0;

% combine RPMs
N = [N_LP;N_MP];

%% Form Control Effector Command Vector (DO NOT EDIT)

U = [...
    da1;    % 1
    da2;    % 2
    dr1;    % 3
    dr2;    % 4
    N;      % 5-8,9
    ];

end
