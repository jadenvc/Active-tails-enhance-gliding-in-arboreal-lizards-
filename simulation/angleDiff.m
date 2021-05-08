function [angleout] =angleDiff(anglein)
    %Jaden Clark
    %Stanford University
    %jvclark@stanford.edu
    %7 May 2021

    %this file contains code to normalize angle between pi and -pi

    angleout = anglein;

    while angleout > pi
        angleout = angleout -2*pi;
    end

    while angleout < -pi
        angleout = angleout + 2*pi;
end

