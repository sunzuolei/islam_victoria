function [z, H] = obsModel(x, idf)
    % Inputs:
    %   x   - state vector.
    %   idf - index of observations.
    % Outputs:
    %   z   - predicted observation
    %   H   - jacobian about observation.
   %%
    Nxv  = 3;                  % Dimension of robot pose.
    fpos = Nxv + idf * 2 - 1;  % Order of observations in global state variance.
     H   = zeros(2, length(x));

    % Auxiliary values
    dx  = x(fpos)  -x(1); 
    dy  = x(fpos+1)-x(2);
    c   = cos(x(3)); s   = sin(x(3));
    rot = [ c s;
           -s c];
       
    %% Predict measurements.
    z =  rot * [dx;dy];
    
    %% Calculate H
    H(:,1:3)         = [-c  -s  -s*dx+c*dy;
                         s  -c  -c*dx-s*dy];
    H(:,fpos:fpos+1) = [ c   s;   -s   c];
end