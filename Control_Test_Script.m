%% File Information
% Control_Test_Script - Populated control allocation example for the Hero VTOL
%
% PURPOSE:
%   Call ControlAllocation for five example cases and display one MATLAB table with
%   cases as COLUMNS and input/output quantities as ROWS. No plots are needed.
%
% USE:
%   First run with the provided ControlAllocation.p in the current folder.
%   All caller inputs are populated below. Record the reference table before
%   completing the four surface expressions and dBmat_LP in your own .m file.
%   To test your ControlAllocation.m, move ControlAllocation.p outside the
%   current folder AND MATLAB path, then rerun this script with the same cases.
%   A same-named .p takes precedence over .m in the same folder. This script
%   refreshes function lookup and prints the file used; verify its extension.
%   MATLAB R2024a-compatible; no additional toolbox is required.
%
% OUTPUTS:
%   ResultsTable, UAll (9-by-nCases), and each effector's command vector remain
%   in the workspace. Units are included in the table row names.
%
% STUDENT NAME: Antonio Chappell

clear;
clc;

%% 1. Flight-Condition Configuration (PROVIDED)

Veh = struct();
Veh.Aero.VTXN = 30;       % Transition airspeed [knots]

% Use this instructor-provided flight condition for every case.
FltCon = struct();
FltCon.ALT = 223;
FltCon.SpeedOfSound = 339.44;
FltCon.rho = 1.199;
FltCon.DynVisc = 1.7824e-05;
FltCon.FPA = 0;
FltCon.KEAS = 0.01;       % Equivalent airspeed [knots]
FltCon.V = 0.0051444;

% The allocator reads only FltCon.KEAS and Veh.Aero.VTXN from these structs.
% At KEAS = 0.01 and VTXN = 30 knots, differential lift-propeller commands
% are fully active. RPM gains and dBmat_LP are inside the allocation function.

%% 2. Example Input Cases
% Each COLUMN is one UTrimVec. All cases use common commands of 5000 RPM
% for the lift propellers and 2000 RPM for the cruise propeller.
%
% Case 1: neutral control inputs.
% Case 2: full positive roll input (ulat = +1).
% Case 3: full positive pitch input (ulon = +1).
% Case 4: full positive yaw input (udir = +1).
% Case 5: combined half-positive roll, pitch, and yaw inputs.
%
%                 Neutral   Roll   Pitch    Yaw   Combined
%                  Case 1  Case 2  Case 3  Case 4  Case 5
UTrimCases = [ ...
                       0,      1,      0,      0,     0.5;  % 1. ulat  [-]
                       0,      0,      1,      0,     0.5;  % 2. ulon  [-]
                       0,      0,      0,      1,     0.5;  % 3. udir  [-]
                    2000,   2000,   2000,   2000,    2000;  % 4. MPRPM [RPM]
                    5000,   5000,   5000,   5000,    5000;  % 5. LPRPM [RPM]
                       0,      0,      0,      0,       0;  % 6. NAC   [deg]
    ];

% Individual lift-propeller outputs differ from the common 5000 RPM command
% when differential roll, pitch, or yaw inputs are applied.

% Optional: add a seventh row of dflp values [deg] for direct flap commands.
% Six rows select the automatic flap schedule. NAC and dflp are retained
% for compatibility; neither changes U in this nine-effector model.

%% 3. Call the Reference or Student Function (PROVIDED)
% The call is the same for .p and .m. Move the .p file out of MATLAB's search
% locations before testing your .m; confirm the printed path ends in .m.

clear ControlAllocation;
rehash;
AllocationFile = which('ControlAllocation');
assert(~isempty(AllocationFile), ...
    'Place the provided ControlAllocation.p or your ControlAllocation.m in the current folder.');
fprintf('Using allocator: %s\n\n',AllocationFile);

nCases = size(UTrimCases,2);
nInputs = size(UTrimCases,1);
assert(nCases >= 3 && nCases <= 5, ...
    'Define 3-5 cases, with one case per column of UTrimCases.');
assert(nInputs == 6 || nInputs == 7, ...
    'UTrimCases must have six rows, or seven if dflp is included.');
assert(isreal(UTrimCases) && all(isfinite(UTrimCases(:))), ...
    'All entries of UTrimCases must be finite, real input values.');

UAll = zeros(9,nCases);
for iCase = 1:nCases
    UTrimVec = UTrimCases(:,iCase);
    U = ControlAllocation(UTrimVec,FltCon,Veh);

    assert(isequal(size(U),[9,1]) && isreal(U) && all(isfinite(U(:))), ...
        'Case %d: U must contain nine finite, real values in a column vector.', ...
        iCase);
    UAll(:,iCase) = U;
end

%% 4. Separate the Effector Commands (PROVIDED)
% Each row vector below contains one command per case.

da1_deg = UAll(1,:);     % Left aileron
da2_deg = UAll(2,:);     % Right aileron
dr1_deg = UAll(3,:);     % Left ruddervator
dr2_deg = UAll(4,:);     % Right ruddervator
N_LP1_RPM = UAll(5,:);   % Front-left lift propeller
N_LP2_RPM = UAll(6,:);   % Front-right lift propeller
N_LP3_RPM = UAll(7,:);   % Rear-left lift propeller
N_LP4_RPM = UAll(8,:);   % Rear-right lift propeller
N_MP1_RPM = UAll(9,:);   % Cruise propeller

%% 5. Create the Results Table (PROVIDED)

InputRowNames = { ...
    'KEAS (knots)';
    'ulat (-)';
    'ulon (-)';
    'udir (-)';
    'MPRPM input (RPM)';
    'LPRPM input (RPM)';
    'NAC (deg)'};
if nInputs == 7
    InputRowNames{end+1,1} = 'dflp (deg)';
end

OutputRowNames = { ...
    'da1 - Left aileron (deg)';
    'da2 - Right aileron (deg)';
    'dr1 - Left ruddervator (deg)';
    'dr2 - Right ruddervator (deg)';
    'N_LP1 (RPM)';
    'N_LP2 (RPM)';
    'N_LP3 (RPM)';
    'N_LP4 (RPM)';
    'N_MP1 (RPM)'};

CaseNames = arrayfun(@(k) sprintf('Case%d',k),1:nCases, ...
    'UniformOutput',false);
RowNames = [InputRowNames; OutputRowNames];
TableData = [repmat(FltCon.KEAS,1,nCases); UTrimCases; UAll];
ResultsTable = array2table(TableData, ...
    'VariableNames',CaseNames,'RowNames',RowNames);
disp(ResultsTable);
