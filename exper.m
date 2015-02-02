

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
joint = Motor('experiments','iCubGenova03','leg','right','ankle','pitch');
% Set number of joint you want control and read
joint = joint.setPart('number_joint',1);

%% Load from file measure of friction
joint = joint.loadIdleMeasure('idle',1,100);

%% Save information in file
joint.saveToFile();

%% Plot Friction
%Counter figures
if ~exist('counter','var')
    counter = 1;
end
% FIGURE - Friction data and estimation
joint.friction.savePictureToFile(joint.path, counter);
counter = counter + 1;
joint.friction  % print information about friction

% FIGURE - Noise on data
hFig = figure(counter);
set(hFig, 'Position', [0 0 800 600]);
hold on
joint.friction.plotNoise();
grid;
hold off
joint.savePictureToFile(hFig,'Noise');
counter = counter + 1;

%Clear variables
%clear counter;
clear hFig;