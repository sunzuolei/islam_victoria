dbstop if error;
clear all; close all;
path(path, genpath('../IEKF and EKF SLAM on VictoriaPark data'));
load 'Data/gps';
M = importdata('Data/victoriaPark.txt');
I = imread('Data/vicPark.bmp','BMP');
%% Switches
openIEKF = 1; % If zero, run EKF-SLAM demo. Otherwise run IEKF-SLAM,
              % and the value of openIEKF is the iterative number.            
%% Parametres configuration
% step     = size(M.data,1);
step     = 10608;
pos      = [0;0;pi/4-0.07];
cov      = zeros(3,3); % Initial covariance.
z        = [];
%% Assign memory and Initialise.
data.path      = zeros(3, 6969); % Assign memory for estimated path.
data.path(:,1) = pos;
data.pos(1).x  = pos;            % State include robot and landmarks.
covAll         =zeros(3,3,6969); % Robot covariance
idList         = zeros(1,6884);  % Assign memory for corresponding list.
%% Noise
tuneQ = 1.0;
tuneW = 1.0;
% Process noise for filter
sigmaXNoise   = 2*1/100;
sigmaYNoise   = 2*1/500;
sigmaPhiNoise = 2*1/500;
     Q        = tuneQ*(diag([sigmaXNoise,sigmaYNoise,sigmaPhiNoise])).^2;
% Measurements noises for filter
sigmaRNoise   = 0.5*1/1.581139;
sigmaBNoise   = 0.5*1/1.581139;
W             = tuneW*(diag([sigmaRNoise, sigmaBNoise])).^2;

%% Initialize figure.
figure('name','SLAM Demo','color','w','units','normalized',...
         'outerposition',[0 0 1 1]);
hold on;box on; axis equal;
image([-142 188],[260 -120],I,'alphadata',1);  % Satelite map
axis([-120 160 -50 240]); 
plot(Lo_m(1:7:end,1)+75,La_m(1:7:end,1)+45, 'b.', 'markersize', 5);  % Ground truth
hold on;
set(gca,'xtick',[],'ytick',[]); 
set(gca,'Color','w','XColor','w','YColor','w')
odoPath     = plot(0,0,'r-','linewidth',1.5,'erasemode','normal');
obsFeature  = plot(0,0,'y+','linewidth',1.5,'markersize',8,'erasemode','normal');

%% Just for legend
truth_       = plot(0,0,'b.','linewidth',3,'markersize',8,'erasemode','normal');
odoPath_     = plot(0,0,'r-','linewidth',1.5,'erasemode','normal');
obsFeature_  = plot(0,0,'y+','linewidth',1.5,'markersize',8,'erasemode','normal');
label        = [odoPath_,truth_, obsFeature_];
lgd          = legend(label,'Estimated Path','GPS','Features');
set(lgd,'box','on','position',[0.61 0.83 0.1 0.1],'color','w',...
    'FontSize',11,'fontname','song','fontweight','bold');

%% Main Loop
disp('Wait for a moment...');
k         = 2; % Delete the predict state in store.
j         = 1;
obsPeriod = 2; % (s). Time interval of Period.
dtSum     = 0;
tic;
for i = 1:step
    if strcmp(M.rowheaders{i}(1:8),'ODOMETRY') 
        [pos, cov]     = predictEKF(pos, cov, M.data(i,3:5)', Q);
        data.path(:,k) = pos(1:3);
        data.pos(k).x  = pos;
        covAll(:,:,k)  = cov(1:3,1:3);
              k        =k+1;   % Delete the predict state
    else
        dtSum     = dtSum + 1;
        if dtSum >= obsPeriod
            dtSum = 0;
        %% Observe 
        [zf, idf, zn, idList] = correspond(pos,M.data(i,2:4)',idList);
        %% Update 
         if openIEKF == 0
             [pos, cov] = updateEKF(pos, cov, zf, W, idf);
         else
             [pos, cov] = updateIEKF(pos, cov, zf, W, idf, openIEKF);
         end
         [pos, cov] = augmentState(pos, cov, zn, W);
        end
         data.path(:,k-1) = pos(1:3);
         data.pos(k-1).x  = pos;
         %% 
         set(odoPath,  'xdata', data.path(1,1:k-1), ...
                 'ydata', data.path(2,1:k-1));
             % Observations
         if  ~isempty(length(pos)-3) 
             set(obsFeature, 'xdata', pos(4:2:end),...
                 'ydata', pos(5:2:end));
         end
         drawnow;
         hold on;
    end
end
toc;