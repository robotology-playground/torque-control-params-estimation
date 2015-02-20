function [hFig, counter] = savePictureKt(coupled, counter)
    axes_data = [];
    for i=1:size(coupled.joint,2)
        coupled.joint(i).savePictureKt(counter);
        axes_data = [axes_data gca];
        counter = counter + 1;
    end
    if ~exist('figureName','var')
        figureName = 'PWMVsTorque';
    end
    hFig = coupled.plotCollect(counter, axes_data, figureName);
    counter = counter + 1;
end