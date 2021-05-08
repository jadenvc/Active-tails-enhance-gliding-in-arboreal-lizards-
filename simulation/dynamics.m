function [drag,lift, aa, cl] = dynamics(vxprev,vyprev,density,area,thetaprev)
    %Jaden Clark
    %Stanford University
    %jvclark@stanford.edu
    %7 May 2021

    %this function computes the lift and drag forces on Draco at one
    %timestep

    aa = angleDiff(thetaprev - atan2(vyprev,vxprev));

    cl = calc_cl(aa);
    cd = calc_cd(aa);


    drag = ((vxprev^2+vyprev^2))*0.5*density*cd*area;
    lift = ((vyprev^2+vxprev^2))*0.5*density*cl*area;
end

