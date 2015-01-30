
name = 'ref';
Reference = struct;

if strcmp(name,'idle')
    Reference.sinAmp = 0;
    Reference.sinFreq = 0;
    Reference.sinBias = 0;
elseif strcmp(name,'ref')
    Reference.sinAmp = 1;
    Reference.sinFreq = 0.05;
    Reference.sinBias = 0;
end

%% Save on file
if exist('tout','var')
    time = tout;
    clear tout;
end
save([joint.path name '.mat'],'logsout','time');
disp(['SAVED ON ' joint.path name '.mat']);

%% Load Reference

joint = joint.loadReference('ref');

joint.plotFrVsMeasure();