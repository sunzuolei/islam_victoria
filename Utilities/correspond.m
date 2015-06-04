function [zf, idf, zn, cList] = correspond(x, z, cList)
    % Maintains a measurement corresponding list. 
    % Adapted from code by Tim Bailey
    % Inputs:
    %    x    - global state covarianc.
    %    z    - observations.
    %   idz   - observations' ID.
    %   cList - corresponding list recoding the observations appearance order.
    % Outputs:
    %  zf,idf - old observations(already add to the list) and its ID.
    %     zn  - new observations.
    %   cList - updated corresponding list.
    %% Store old observations, new observations and their index.
    % Old observations used to update. New observations added to global augment state.
    idz = z(1,:);
    zf  = []; zn  = [];
    idf = []; idn = [];   
    %% Distinguish old and new observations.
    for i  = 1:length(idz)
        ii = idz(i);
        if ii == 0 
            return; 
        end
        if cList(ii) == 0         % New observations.
            zn  = [zn z(2:3,i)];
            idn = [idn ii];       % ID.
        else                      % Old observations.
            zf  = [zf z(2:3,i)];
            idf = [idf cList(ii)];% Index to observation.
        end
    end
    %% Add new features order to corresponding list
    if ~isempty(zn)
        Nxv        = 3;                   
        Nf         = (length(x) - Nxv)/2; % Observations number in map.
        cList(idn) = Nf + (1:size(zn,2)); % Add new observations' index
    end
end   
