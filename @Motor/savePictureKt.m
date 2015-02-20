function [hFig, counter] = savePictureKt(joint, counter)
    %% Save Friction picture
    % FIGURE - Friction data and estimation
    if ~exist('counter','var')
        counter = 1;
    end
    hFig = figure(counter);
    set(hFig, 'Position', [0 0 800 600]);
    hold on
    grid;
    joint.plotKt();
    hold off
    % Save image
    currentFolder = pwd;
    cd(joint.path);
    if ~exist('figureName','var')
        figureName = 'PWMVsTorque';
    end
    saveas(hFig,[figureName '.fig'],'fig');
    saveas(hFig,[figureName '.png'],'png');
    cd(currentFolder);
    counter = counter + 1;
end