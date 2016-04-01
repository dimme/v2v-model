function G = V2V_antresp( p, channel, angleTx, angleRx )
% V2V_antresp ... returns the antenna respose for a certain Tx AND Rx angle
%
% Parameters:
% p         V2V parameter structure containing the antenna response and the
%           antenna rotations
% channel   containing the channel number (1...N_Tx*N_Rx)
% angleTx   contains a vector of Tx angles
% angleRx   contains a vector of corresponding Rx angles
%
% Returns:
% Antenna gain vector for the combined Tx/Rx angles
%
% History:
% 2008-03-13 NCZ: created

% Select the angular indices;    
switch channel
    case 1, phiTx = p.phi(1,:); phiRx = p.phi(1,:);
       
    case 2, phiTx = p.phi(2,:); phiRx = p.phi(1,:);
   
    case 3, phiTx = p.phi(1,:); phiRx = p.phi(2,:);
    
    case 4, phiTx = p.phi(2,:); phiRx = p.phi(2,:);
end

[~,idxTx] = min(abs(repmat(angleTx.'/pi*180,1,length(phiTx)) - repmat(phiTx,length(angleTx),1)),[],2);
[~,idxRx] = min(abs(repmat(angleRx.'/pi*180,1,length(phiRx)) - repmat(phiRx,length(angleRx),1)),[],2);

G = p.G_ant(idxTx) .* p.G_ant(idxRx);
    
    
    

