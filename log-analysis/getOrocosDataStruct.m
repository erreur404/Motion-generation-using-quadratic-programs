function [report skip]= getOrocosDataStruct(filename, numjoints)
    joints = 1:numjoints;
    [data skip] = readOrocosData(filename, 2+4*numjoints);
    
    report = struct;
    report.data = data;
    idx=0;
    report.timestamps                      = data(:,1+idx);%special
    idx=idx+1;
    report.feedback_angles                 = data(:,joints+idx);
    idx=idx+numjoints;
    report.feedback_velocities             = data(:,joints+idx);
    idx=idx+numjoints;
    report.feedback_torques                = data(:,joints+idx);
    idx=idx+numjoints;
    report.command_torques                 = data(:,joints+idx);
    idx=idx+numjoints;

    % not sure about this vv
    %assert(idx+1==size(data,2)); %+1 because of last space as senseless variable
end
