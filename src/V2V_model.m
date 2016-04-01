function V2V_model( p )
% V2V_model  --- Optimised Vehicle-to-Vehicle model
% 
% Based on the C2C model of Johan Karedal, Lund University, Sweden
% Optimised by Nicolai Czink, FTW, Vienna, Austria
%
% Input paramters: 
% params ... V2V parameter structure 
%            OR
%            filename containing the parameters (NOT in a structure)
%              (save a params structure with "save <filename> -struct <structurename>")
% 
% Output: 
% This function writes a file containing the modelled snapshots of the
% channel and the scenario parameters (filename specified in the params 
% structure)
%
% Quick-Start-Guide:
% Call
% V2V_model_opt(  ) 
% for opposite direction highway scenario @ 90 km/h
%
% OR
% 
% V2V_model_opt( V2V_model_params('SD') ) 
% for same direction highway scenario @ 90 km/h
%
% History:
% 2008-03-13 NCZ: created

if nargin == 0,
    p = V2V_model_params;
end

if ischar(p),
    p = load(p);
end

save(p.filename,'p');

%% Determine distance vectors and angle vectors
% New implementation: all time instants are calculated at the same time
% while the loops are over the antenna elements and the scatterers
% -> speed increases by a factor of ~60.

fprintf('Determining distance vectors ... \r\n');

n_T = length(p.T);

% Preliminary matrix/vector stacking
d_LOS    = zeros(p.N_Tx*p.N_Rx,  n_T);
d_MD     = zeros(p.N_Tx*p.N_Rx,  p.N_MD,     n_T);
d_SD     = zeros(p.N_Tx*p.N_Rx,  p.N_SD,     n_T);
d_D      = zeros(p.N_Tx*p.N_Rx,  p.N_D,      n_T);
thTxRx   = zeros(p.N_Tx*p.N_Rx,  n_T, 'single');
thTxMD   = zeros(p.N_Tx*p.N_Rx,  p.N_MD, n_T, 'single');
thTxSD   = zeros(p.N_Tx*p.N_Rx,  p.N_SD, n_T, 'single');
thTxD    = zeros(p.N_Tx*p.N_Rx,  p.N_D,  n_T, 'single');

thRxMD   = zeros(p.N_Tx*p.N_Rx,  p.N_MD, n_T, 'single');
thRxSD   = zeros(p.N_Tx*p.N_Rx,  p.N_SD, n_T, 'single');
thRxD    = zeros(p.N_Tx*p.N_Rx,  p.N_D,  n_T, 'single');

xTx      = repmat(p.xTx0(:), 1, n_T) + p.vTx(:)*p.T;
xRx      = repmat(p.xRx0(:), 1, n_T) + p.vRx(:)*p.T;
xMD      = repmat(p.xMD0(:), 1, n_T) + p.vMD(:)*p.T;
xSD      = repmat(p.xSD(:),  1, n_T);
xD       = repmat(p.xD(:),   1, n_T);

% Determining distances and angles
ctr_ch = 0;
for ch2 = 1:p.N_Rx % for all channels
    for ch1 = 1:p.N_Tx
        ctr_ch = ctr_ch + 1;
        
        % Determine Tx and Rx channel
        xTx_ch = xTx([ch1; ch1 + p.N_Tx],:);
        xRx_ch = xRx([ch2; ch2 + p.N_Rx],:);
        
        % Determine LOS distance and angle
        d_LOS(ctr_ch,:) = sqrt( sum((xTx_ch - xRx_ch).^2,1) );
        thTxRx(ctr_ch,:) = acos((xRx_ch(1,:)-xTx_ch(1,:))./d_LOS(ctr_ch,:));
        
        % Determine SD distances and angles
        for ctr_SD = 1:p.N_SD
            xSD_this = xSD([ctr_SD; ctr_SD + p.N_SD],:);
            toSD = sqrt( sum((xTx_ch - xSD_this).^2,1) );
            fromSD = sqrt( sum((xSD_this - xRx_ch).^2,1));
            d_SD(ctr_ch,ctr_SD,:) =  toSD + fromSD;
            
            thTxSD(ctr_ch,ctr_SD,:) = acos((xSD_this(1,:)-xTx_ch(1,:))./toSD);
            thRxSD(ctr_ch,ctr_SD,:) = acos((xSD_this(1,:)-xRx_ch(1,:))./fromSD);
        end
     
        % Determine MD distances and angles
        for ctr_MD = 1:p.N_MD
            xMD_this = xMD([ctr_MD; ctr_MD + p.N_MD],:);
            toMD = sqrt( sum((xTx_ch - xMD_this).^2,1) );
            fromMD = sqrt( sum((xMD_this - xRx_ch).^2,1));
            d_MD(ctr_ch,ctr_MD,:) =  toMD + fromMD;
            
            thTxMD(ctr_ch,ctr_MD,:) = acos((xMD_this(1,:)-xTx_ch(1,:))./toMD);
            thRxMD(ctr_ch,ctr_MD,:) = acos((xMD_this(1,:)-xRx_ch(1,:))./fromMD);
        end

        % Determine diffuse distances and angels
        for ctr_D = 1:p.N_D
            xD_this = xD([ctr_D; ctr_D + p.N_D],:);
            toD =  sqrt( sum((xTx_ch - xD_this).^2,1) );
            fromD = sqrt( sum((xD_this - xRx_ch).^2,1));
            d_D(ctr_ch,ctr_D,:) =  toD + fromD;
            
            thTxD(ctr_ch,ctr_D,:) = acos((xD_this(1,:)-xTx_ch(1,:))./toD);
            thRxD(ctr_ch,ctr_D,:) = acos((xD_this(1,:)-xRx_ch(1,:))./fromD);
        end
    end
end

if 0; plotTheoTimeDelay(p, d_LOS, d_MD, d_SD); end

%% Generate LS fading

[G_LS_LOS, G_LS_MD, G_LS_SD] = V2V_gen_LS_fading( p, d_LOS, d_MD, d_SD );

%% Generate channel matrix
fprintf('\nGenerate channel matrix ...\n')
ctr_t = 0;
H = zeros(p.chunksize,p.N_Rx*p.N_Tx,length(p.F));
chunk = 0;
save_cell = cell(floor(length(p.T)/p.chunksize), 2);

cpb = CreateProgressBar(length(p.T));
cpb.start();
for t = p.T
    ctr_t = ctr_t + 1;
    idx_t = mod(ctr_t,p.chunksize);
    
    userText = sprintf('Progress: [%d/%d]', ctr_t, length(p.T)); 
    cpb.setValue(ctr_t); % update progress value 
    cpb.setText(userText) % update user text 
    
    % overcome the MATLAB non-zero indexing
    if idx_t == 0; idx_t = p.chunksize; end;
        
    % Add LOS component
    for ctr_ch = 1:p.N_Rx*p.N_Tx;

        % LOS path
        H(idx_t,ctr_ch, :) = H(idx_t, ctr_ch, :) + ...
                       shiftdim((V2V_antresp(p, ctr_ch, thTxRx(ctr_ch,ctr_t), thTxRx(ctr_ch,ctr_t)+pi).* ... 
                         10.^(p.G0_LOS/20) .* 10.^(G_LS_LOS(ctr_t)/20) .* exp(-1j*2*pi*p.F/p.c*d_LOS(ctr_ch,ctr_t)) ./ ...
                         (d_LOS(ctr_ch,ctr_t)^(p.n_PL_LOS/2))),-1);
        % MD paths
        H(idx_t, ctr_ch, :) = H(idx_t, ctr_ch, :) + ...
            shiftdim((V2V_antresp(p, ctr_ch,thTxSD(ctr_ch, :, ctr_t),thRxSD(ctr_ch, :, ctr_t)).* ... % SD
            10.^(p.G0_SD.'/20) .* 10.^(G_LS_SD(:,ctr_t).'/20) ./ ((d_SD(ctr_ch,:,ctr_t).^(p.n_PL_SD(:).'/2)))) ...
            * exp(-1j*2*pi/p.c*d_SD(ctr_ch,:,ctr_t).'*p.F),-1);
        % SD paths
        H(idx_t, ctr_ch, :) = H(idx_t, ctr_ch, :) + ...
            shiftdim((V2V_antresp(p, ctr_ch,thTxMD(ctr_ch, :, ctr_t),thRxMD(ctr_ch, :, ctr_t)).* ... % MD
            10.^(p.G0_MD.'/20) .* 10.^(G_LS_MD(:,ctr_t).'/20) ./ ((d_MD(ctr_ch,:,ctr_t).^(p.n_PL_MD(:).'/2))) ) ...
            * exp(-1j*2*pi/p.c*d_MD(ctr_ch,:,ctr_t).'*p.F),-1);
        % Diffuse paths
        H(idx_t, ctr_ch, :) = H(idx_t, ctr_ch, :) + ...
            shiftdim( ( V2V_antresp(p, ctr_ch, thTxD(ctr_ch, :, ctr_t),thRxD(ctr_ch, :, ctr_t)).* ... % D
            10.^(p.G0_D/20) * (1/sqrt(p.N_D)) ./d_D(ctr_ch,:,ctr_t).^(p.n_PL_D/2) ) ...
            * exp(-1j*2*pi/p.c* d_D(ctr_ch,:,ctr_t).'*p.F), -1);
    end
    
    
    % Store a chunk when it is ready
    if idx_t == p.chunksize,
        chunk = chunk + 1;
        eval( sprintf( 'fr%09d.H = H;', floor(ctr_t/p.chunksize)) );
        eval( sprintf( 'fr%09d.t = p.T(ctr_t-p.chunksize+1);', floor(ctr_t/p.chunksize)) );
        save_cell{chunk, 1} = sprintf( 'fr%09d', floor(ctr_t/p.chunksize) );
        save_cell{chunk, 2} = eval(sprintf( 'fr%09d', floor(ctr_t/p.chunksize) ));
        eval( sprintf( 'clear(''fr%09d'')', floor(ctr_t/p.chunksize)) );
        H = zeros(p.chunksize,p.N_Rx*p.N_Tx,length(p.F));
    end
    
end
cpb.stop();

c = cell2struct(save_cell(:,2), save_cell(:,1));

hSize = whos('c');

fprintf('\n   ... saving channel matrix (compressing %4.2f MB) ...', hSize.bytes*1e-6);
save('-append', p.filename, '-struct', 'c');

% do the last write if it hasn't been done before.
if idx_t ~= p.chunksize,
    H = H(1:idx_t,:,:);
    eval( sprintf( 'fr%09d.H = H;', ceil(ctr_t/p.chunksize)) );
    eval( sprintf( 'fr%09d.t = p.T(ctr_t-p.chunksize+1);', ceil(ctr_t/p.chunksize)) );
    save( '-append', p.filename, sprintf( 'fr%09d', ceil(ctr_t/p.chunksize) ) );
    eval( sprintf( 'clear(''fr%09d'')', ceil(ctr_t/p.chunksize)) );
end

MyFileInfo = dir(p.filename);
fprintf('\nFinished! Output saved to:\n\t%s (%4.2f MB)\n', p.filename, MyFileInfo.bytes*1e-6)

