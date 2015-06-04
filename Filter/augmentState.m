function [x, P] = augmentState(x, P, z, R)
    % Inputs:
    %   z, R - range-bearing observations(new) and its covariance.
    % Outputs:
    %   x, P - global state and covariance.
    %%
    for i = 1:size(z,2)
        lenx = length(x);
        lx   = z(1,i); 
        ly   = z(2,i);
        %
        s    = sin(x(3)); 
        c    = cos(x(3));
       %% Jacobians
        Gv = [1 0  -lx*s-ly*c;
              0 1  lx*c-ly*s];
        Gz = [c   -s;
              s    c];
       %% Augment state
         x = [x;
              x(1) + lx*c - ly*s;
              x(2) + lx*s + ly*c];
       %% Augment covariance
        rng        = lenx+1:lenx+2;
        P(rng,rng) = Gv * P(1:3,1:3) * Gv' + Gz * R * Gz';
        P(rng,1:3) = Gv * P(1:3,1:3); % Covariance between robot and observations.
        P(1:3,rng) = P(rng,1:3)';
        if lenx>3
            rnm        = 4:lenx;
            P(rng,rnm) = Gv * P(1:3,rnm);
            P(rnm,rng) = P(rng,rnm)';
        end
    end
end
