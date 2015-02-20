function [hFig, counter] = savePictureFriction(joint, counter)
    name = [upper(joint.part) ' ' upper(joint.type) ' ' upper(joint.info1) ' ' upper(joint.info2)];
    hFig = joint.friction.savePictureToFile(joint.path, name, counter);
    counter = counter + 1;
end