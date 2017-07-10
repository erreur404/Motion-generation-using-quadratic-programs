% function logAnalysis()
    clc;
    close all;
    clear all;

    numjoints = 7;
    [report  skip]  = getOrocosDataStruct(['../ops/reports.dat'], numjoints);

    mystarttime = 1.0;
    mystoptime  = 5.0;
    mystarttime = report.timestamps(1);
    mystoptime  = report.timestamps(end);
    idxStart=find(report.timestamps>=mystarttime,1);
    idxStop=find(report.timestamps>=mystoptime,1);
    idxArea=idxStart:idxStop;
    report.timestampsArea = report.timestamps(idxArea);
    report.timestampsArea = report.timestampsArea - mystarttime;
    xLimit = [0.0, mystoptime - mystarttime];

    for jointID=1:numjoints
        fig=figure();
        hold all;
        title(['Joint ' num2str(jointID)])
        plot(report.timestampsArea, report.feedback_angles(idxArea,jointID), '-g');
        xlabel('Time [sec]')
        ylabel('Angle [rad]')
        xlim(xLimit)
    end
    
    for jointID=1:numjoints
        fig=figure();
        hold all;
        title(['Joint ' num2str(jointID)])
        plot(report.timestampsArea, report.command_torques(idxArea,jointID), '-g');
        xlabel('Time [sec]')
        ylabel('Torque [Nm]')
        xlim(xLimit)
    end
