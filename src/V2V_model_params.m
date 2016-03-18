% V2V_model_params 
%       generates a parameter set for the optimised V2V model
%       
%       Input parameters:
%       dir  ...  When empty, or 'OD', a highway, opposite-direction scenario is parametrised
%                 When 'SD', a highway, same-direction scenario is parametrised
%
%       Output parameters:
%       params .. V2V model parameters
%                 NOTE: Set params.filename to the output file path!
%
%       Prerequisites: 
%       Make sure that Eaz.mat is in the current directory or in the MATLAB
%       path

% History:
% 2008-03-13 NCZ: created

function p = V2V_model_params( dir )

rootDir = fullfile(fileparts(fileparts(which(mfilename))));

p.filename = fullfile(rootDir,'V2V_model_output.mat');
p.chunksize = 10;

if nargin == 1,
    if strcmp( dir,'SD')
        p.di = 'SD';
    else
        p.di = 'OD';
    end
else
    p.di = 'OD';
end
        

%% Radio parameters
% dt    Measured value 0.3072e-3;
% Tmax  Measured value 10 s250
p.dt    = 0.3072e-3; %.01;
p.Tmax  = 0.5;
p.T     = 0:p.dt:p.Tmax;
p.F     = (5.08:0.01/32:5.32)*1e9;

%% Geometry limits    - geometric coordinates organized as matrices "x" with {x_coord ycoord} as rows
p.xmin = [-250 0];
p.xmax = [250 250];
p.w_road = 17;            % Width of road
p.NbrLanes = 4;           % Number of lanes, must be even
p.y_m_SD = 14;            % Distance from center of road to mean value of SD positions
p.y_sigma_SD = 3;         % Standard deviation of SD position y-coordinates
p.y_m_D = 11;             % Distance from center of road to first value of diffuse scattering positions
p.w_D = 5;                % Width of diffuse scattering band

%% Statistics
% Scatterer densities
p.chi_SD = 0.005;
p.chi_MD = 0.005;
p.chi_D = 1;

% Distribution parameters for model parameters
p.mu_sigma2_LOS = 6.8;    p.mu_sigma2_MD = 9.4;   p.mu_sigma2_SD = 6.3;
p.mu_d50_LOS = 7.2;       p.mu_d50_MD = 5.4;    p.mu_d50_SD = 4.9;
p.d50_min_LOS = 4.4;      p.d50_min_MD = 1.1;   p.d50_min_SD = 1.0;
p.n_PL_LOS = 1.6;         p.m_n_MD = 1.9;       p.m_n_SD = 1.0;           p.n_PL_D = 8;
p.sigma_n_MD = 2.5;   p.sigma_n_SD = 3.1;
p.n_min_MD = 0;       p.n_min_SD = 0;
p.n_max_MD = 3.5;       p.n_max_SD = 3.5;
p.m_v_MD = 90;
p.sigma_v_MD = 2;
p.G0_LOS = -30;                                                       p.G0_D = 100;

% Implementation parameters
p.L_filter = 500;

% Time, frequency vectors etc.
p.c = 3e8;
p.NbrFreq = length(p.F);
p.BW = p.F(end)-p.F(1);
p.tau = (0:p.NbrFreq-1)/p.BW;
p.nu = ((0:length(p.T)-1)/p.dt/(length(p.T))); p.nu = p.nu-p.nu(end)/2;


% Antenna patterns
Eaz = [];
load(fullfile(rootDir,'res','antenna','Eaz.mat'));
[~,ind_max] = max(abs(Eaz));
p.G_ant = single([Eaz(ind_max:end) Eaz(1:ind_max-1)]);
p.phi(1,:) = [224:2:358 0:2:222];
p.phi(2,:) = [314:2:358 0:2:312];
p.phi(3,:) = [44:2:358 0:2:42];
p.phi(4,:) = [134:2:358 0:2:132];

p.xTx0 = [ 0.00  0.00 % Tx initial position vector [xcoord,ycoord], el 1
           0.09  0.00];                       %                                           el 2
p.xRx0 = [20.00  0.00                       % Rx    "        "      "          "
          20.09  0.00];
p.N_Tx = 2;
p.N_Rx = 2;
p.vTx = repmat([90 0],p.N_Tx,1)/3.6;                   % Tx velocity vector [xcoord,ycoord]
p.vRx = repmat([91 0],p.N_Rx,1)/3.6;                   % Rx    "        "         "
if strcmp(p.di,'OD')
    p.xRx0(:,2) = p.w_road - p.w_road/p.NbrLanes;
    p.vRx = -p.vRx;
else
    p.xRx0(:,2) = 0;
end

%%% Generate a random environment
fprintf( 'Generating random environment ...\r\n');

p.N_SD = ceil(p.chi_SD*(p.xmax(1)-p.xmin(1))/2)*2;
p.N_MD = round(randn + p.chi_MD*(p.xmax(1)-p.xmin(1))); if p.N_MD < 0, p.N_MD = 0; end;
p.N_D = p.chi_D*(p.xmax(1)-p.xmin(1));

p.xMD0 = [(p.xmax(1)-p.xmin(1))*rand(p.N_MD,1)+p.xmin(1) p.w_road/p.NbrLanes*(ceil(p.NbrLanes*rand(p.N_MD,1))-1)];
p.vMD = [p.sigma_v_MD*randn(p.N_MD,1)+p.m_v_MD zeros(p.N_MD,1)]/3.6;
for ctr_MD = 1:p.N_MD
    if p.xMD0(ctr_MD,2)/p.w_road >= .5 % Other lane => other direction
        p.vMD(ctr_MD,1) = - p.vMD(ctr_MD,1);
    end
end

p.xSD = [(p.xmax(1)-p.xmin(1))*rand(p.N_SD/2,1)+p.xmin(1) p.y_sigma_SD*randn(p.N_SD/2,1)+(p.w_road*(p.NbrLanes-1)/2/p.NbrLanes)+p.y_m_SD
    (p.xmax(1)-p.xmin(1))*rand(p.N_SD/2,1)+p.xmin(1) p.y_sigma_SD*randn(p.N_SD/2,1)+(p.w_road*(p.NbrLanes-1)/2/p.NbrLanes)-p.y_m_SD];
p.xD = [(p.xmax(1)-p.xmin(1))*rand(p.N_D/2,1)+p.xmin(1) p.w_D*rand(p.N_D/2,1)+(p.w_road*(p.NbrLanes-1)/2/p.NbrLanes)+p.y_m_D
    (p.xmax(1)-p.xmin(1))*rand(p.N_D/2,1)+p.xmin(1) -p.w_D*rand(p.N_D/2,1)+(p.w_road*(p.NbrLanes-1)/2/p.NbrLanes)-p.y_m_D];

p.sigma_LS_LOS = abs(sqrt(p.mu_sigma2_LOS/2)*(randn+1j*randn)).^2;
p.d05_LOS = p.d50_min_LOS + abs(sqrt(p.mu_d50_LOS/2)*(randn+1j*randn)).^2;

p.sigma_LS_MD = abs(sqrt(p.mu_sigma2_MD/2).*(randn(p.N_MD,1)+1j*randn(p.N_MD,1))).^2;
p.d05_MD = p.d50_min_MD + abs(sqrt(p.mu_d50_MD/2).*(randn(p.N_MD,1)+1j*randn(p.N_MD,1))).^2;
p.n_PL_MD = p.sigma_n_MD*randn(p.N_MD,1) + p.m_n_MD; p.n_PL_MD(p.n_PL_MD < p.n_min_MD) = p.n_min_MD; p.n_PL_MD(p.n_PL_MD > p.n_max_MD) = p.n_max_MD;
p.G0_MD = -100+24*p.n_PL_MD;
p.phi_MD = 2*pi*rand(p.N_MD,1);

p.sigma_LS_SD = abs(sqrt(p.mu_sigma2_SD/2).*(randn(p.N_SD,1)+1j*randn(p.N_SD,1))).^2;
p.d05_SD = p.d50_min_SD + abs(sqrt(p.mu_d50_SD/2).*(randn(p.N_SD,1)+1j*randn(p.N_SD,1))).^2;
p.n_PL_SD = p.sigma_n_SD*randn(p.N_SD,1) + p.m_n_SD; p.n_PL_SD(p.n_PL_SD < p.n_min_SD) = p.n_min_SD; p.n_PL_SD(p.n_PL_SD > p.n_max_SD) = p.n_max_SD;
p.G0_SD = -100+24*p.n_PL_SD;
p.phi_SD = 2*pi*rand(p.N_SD,1);

