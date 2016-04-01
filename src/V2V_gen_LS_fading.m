function [G_LS_LOS, G_LS_MD, G_LS_SD] = V2V_gen_LS_fading( p, d_LOS, d_MD, d_SD )
% V2V_gen_LS_fading
%
% Subroutine for V2V model, generates the LS fading. 
% Speed improvement by using SVDS function (evaluates only the first 6
% singular values, which is much faster and - in this application - equally accurate.
%
% First, samples are generated with equidistant spacing, then the samples
% are linearly interpolated for the (non-equidistant) path lengths
%
% Parameters: 
% p        V2V parameter structure
% d_LOS    LOS distances
% d_MD     MD path distances (for all N_MD paths)
% d_SM     SD path distances (for all N_SD paths)
%
% Output:
% G_LS_LOS, G_LS_MD, G_LS_SD   large-scale fading factors (log-scale!)

% History:
% 2008-03-13 NCZ: created

disp('Generate LS fading')

disp('   ...for LOS...')
d_sim_LOS = [0 cumsum(abs(diff(d_LOS(1,:))))];                             % Correlated data sampled at d_samp_LOS, resample to d_sim_LOS
d_samp_LOS = (0:p.L_filter-1)/(p.L_filter-1)*ceil(d_sim_LOS(end));
G_LS_LOS_uncorr = randn(length(d_samp_LOS),1);
R_LOS_row = p.sigma_LS_LOS^2*exp(-log(2)/p.d05_LOS^2*(d_samp_LOS).^2);
R_LOS = toeplitz(R_LOS_row); [U_LOS,S_LOS,V_LOS] = svds(double(R_LOS));
G_LS_LOS_eqd = ((U_LOS*sqrt(S_LOS)*V_LOS'*G_LS_LOS_uncorr).');

disp('   ...for mobile discrete scatterers...')
d_MD_mean = squeeze(d_MD(1,:,:));
for ctr_MD = 1:p.N_MD
    d_diff_MD = [0 d_MD_mean(ctr_MD,:)]-[d_MD_mean(ctr_MD,:) 0]; d_diff_MD = d_diff_MD(2:end-1);
    d_sim_MD_temp = [0 cumsum(abs(d_diff_MD))];                             % Correlated data sampled at d_samp_MD, resample to d_sim_MD
    d_sim_MD_max(ctr_MD) = d_sim_MD_temp(end);
end
d_samp_MD = (0:p.L_filter-1)/(p.L_filter-1)*ceil(max(d_sim_MD_max));
for ctr_MD = 1:p.N_MD
    G_LS_MD_uncorr = randn(length(d_samp_MD),1);
    R_MD_row = p.sigma_LS_MD(ctr_MD)^2*exp(-log(2)/p.d05_MD(ctr_MD)^2*(d_samp_MD).^2);
    R_MD = toeplitz(R_MD_row); [U_MD,S_MD,V_MD] = svds(double(R_MD));
    G_LS_MD_eqd(ctr_MD,:) = ((U_MD*sqrt(S_MD)*V_MD'*G_LS_MD_uncorr).');
end

disp('   ...for stationary discrete scatterers...')
d_SD_mean = squeeze(d_SD(1,:,:));
for ctr_SD = 1:p.N_SD
    d_diff_SD = [0 d_SD_mean(ctr_SD,:)]-[d_SD_mean(ctr_SD,:) 0]; d_diff_SD = d_diff_SD(2:end-1);
    d_sim_SD_temp = [0 cumsum(abs(d_diff_SD))];                             % Correlated data sampled at d_samp_SD, resample to d_sim_SD
    d_sim_SD_max(ctr_SD) = d_sim_SD_temp(end);
end
d_samp_SD = (0:p.L_filter-1)/(p.L_filter-1)*ceil(max(d_sim_SD_max));
for ctr_SD = 1:p.N_SD
    G_LS_SD_uncorr = randn(length(d_samp_SD),1);
    R_SD_row = p.sigma_LS_SD(ctr_SD)^2*exp(-log(2)/p.d05_SD(ctr_SD)^2*(d_samp_SD).^2);
    R_SD = toeplitz(R_SD_row); [U_SD,S_SD,V_SD] = svds(double(R_SD));
    G_LS_SD_eqd(ctr_SD,:) = ((U_SD*sqrt(S_SD)*V_SD'*G_LS_SD_uncorr).');
end


disp(' Resampling large-scale data...')
disp('   ...for LOS...')
G_LS_LOS(1) = G_LS_LOS_eqd(1);
for ctr_LOS = 2:length(d_sim_LOS)
    xtemp = d_samp_LOS(abs(d_sim_LOS(ctr_LOS)-d_samp_LOS)==min(abs(d_sim_LOS(ctr_LOS)-d_samp_LOS)));
    if length(xtemp) == 2
        x1 = min(xtemp);
        x2 = max(xtemp);
        y1 = G_LS_LOS_eqd(abs(d_samp_LOS-x1)<1e-6);
        y2 = G_LS_LOS_eqd(abs(d_samp_LOS-x2)<1e-6);
    elseif d_sim_LOS(ctr_LOS) < xtemp
        x2 = xtemp;
        y2 = G_LS_LOS_eqd(abs(d_samp_LOS-x2)<1e-6);
        x1 = xtemp - (d_samp_LOS(2) - d_samp_LOS(1));
        y1 = G_LS_LOS_eqd(abs(d_samp_LOS-x1)<1e-6);
    else
        x1 = xtemp;
        y1 = G_LS_LOS_eqd(abs(d_samp_LOS-x1)<1e-6);
        x2 = xtemp + (d_samp_LOS(2) - d_samp_LOS(1));
        y2 = G_LS_LOS_eqd(abs(d_samp_LOS-x2)<1e-6);
    end
    G_LS_LOS(ctr_LOS) = interp1([x1 x2], [y1 y2], d_sim_LOS(ctr_LOS), 'linear', 'extrap');
end

disp('   ...mobile discrete scatterers...')
for ctr_MD = 1:p.N_MD
    d_diff_MD = [0 d_MD_mean(ctr_MD,:)]-[d_MD_mean(ctr_MD,:) 0]; d_diff_MD = d_diff_MD(2:end-1);
    d_sim_MD = [0 cumsum(abs(d_diff_MD))];                             % Correlated data sampled at d_samp_MD, resample to d_sim_MD
    G_LS_MD(ctr_MD,1) = G_LS_MD_eqd(ctr_MD,1);
    for ctr_d = 2:length(d_sim_MD)
        xtemp = d_samp_MD(abs(d_sim_MD(ctr_d)-d_samp_MD)==min(abs(d_sim_MD(ctr_d)-d_samp_MD)));
        if d_sim_MD(ctr_d) < xtemp
            x2 = xtemp;
            y2 = G_LS_MD_eqd(ctr_MD,abs(d_samp_MD-x2)<1e-6);
            x1 = xtemp - (d_samp_MD(2) - d_samp_MD(1));
            y1 = G_LS_MD_eqd(ctr_MD,abs(d_samp_MD-x1)<1e-6);
        else
            x1 = xtemp;
            y1 = G_LS_MD_eqd(ctr_MD,abs(d_samp_MD-x1)<1e-6);
            x2 = xtemp + (d_samp_MD(2) - d_samp_MD(1));
            y2 = G_LS_MD_eqd(ctr_MD,abs(d_samp_MD-x2)<1e-6);
        end
        G_LS_MD(ctr_MD,ctr_d) = interp1([x1 x2], [y1 y2], d_sim_MD(ctr_d), 'linear', 'extrap');
    end
end

disp('   ...stationary discrete scatterers...')
for ctr_SD = 1:p.N_SD
    d_diff_SD = [0 d_SD_mean(ctr_SD,:)]-[d_SD_mean(ctr_SD,:) 0]; d_diff_SD = d_diff_SD(2:end-1);
    d_sim_SD = [0 cumsum(abs(d_diff_SD))];                             % Correlated data sampled at d_samp_SD, resample to d_sim_SD
    G_LS_SD(ctr_SD,1) = G_LS_SD_eqd(ctr_SD,1);
    for ctr_d = 2:length(d_sim_SD)
        xtemp = d_samp_SD(abs(d_sim_SD(ctr_d)-d_samp_SD)==min(abs(d_sim_SD(ctr_d)-d_samp_SD)));
        if d_sim_SD(ctr_d) < xtemp
            x2 = xtemp;
            y2 = G_LS_SD_eqd(ctr_SD,abs(d_samp_SD-x2)<1e-6);
            x1 = xtemp - (d_samp_SD(2) - d_samp_SD(1));
            y1 = G_LS_SD_eqd(ctr_SD,abs(d_samp_SD-x1)<1e-6);
        else
            x1 = xtemp;
            y1 = G_LS_SD_eqd(ctr_SD,abs(d_samp_SD-x1)<1e-6);
            x2 = xtemp + (d_samp_SD(2) - d_samp_SD(1));
            y2 = G_LS_SD_eqd(ctr_SD,abs(d_samp_SD-x2)<1e-6);
        end
        G_LS_SD(ctr_SD,ctr_d) = interp1([x1 x2], [y1 y2],d_sim_SD(ctr_d), 'linear', 'extrap');
    end
end