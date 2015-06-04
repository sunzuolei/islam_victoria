function [x, P] = updateEKF(x, P, z, R, idf)  
    % Inputs:
    %   z, R - range-bearing measurements(old) and their covariances.
    %   idf  - feature ID for each z.
    % Outputs:
    %   x, P - the posterior state and covariance.
%% Single update.   
 if ~isempty(z)
    [zp,H] = obsModel(x, idf); 
      Y    = [z(1) - zp(1);
              (z(2) - zp(2))];
    %% update
    S  = H * P * H' + R;
    K  = P * H' / S;
    x  = x + K * Y; 
    P  = P - K * S * K';
 end