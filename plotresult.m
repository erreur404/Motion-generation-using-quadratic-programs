close all;
clear all;

JOINT_OP = true;

%datalog = importdata("datalog.txt");

filenameOLD = "build/reports.dat";
filenameNEW = 'tmp.dat';


fidOLD = fopen(filenameOLD);
fidNEW = fopen(filenameNEW,'w');
tlineOLD = fgets(fidOLD);
skip = 0;
while ischar(tlineOLD)
    tlineNEW = regexprep(tlineOLD, '[ ]+', ' ');
    l=length(strfind(tlineNEW,' '));
    if(true)
        fwrite(fidNEW,tlineNEW);
    else
        skip = skip+1;
    end
    tlineOLD = fgets(fidOLD);
end
fclose(fidOLD);
fclose(fidNEW);

data = dlmread(filenameNEW, ' ', 1, 0);
delete(filenameNEW);

datalog = data;


% =====================================

display_x = 3;
display_y = 3;
display_index = 1;

col = 1;
t = datalog(:,col);col += 1;

desPosx = datalog(:,col);col += 1;
desPosy = datalog(:,col);col += 1;
desPosz = datalog(:,col);col += 1;
desVelx = datalog(:,col);col += 1;
desVely = datalog(:,col);col += 1;
desVelz = datalog(:,col);col += 1;
desAccx = datalog(:,col);col += 1;
desAccy = datalog(:,col);col += 1;
desAccz = datalog(:,col);col += 1;
curPosx = datalog(:,col);col += 1;
curPosy = datalog(:,col);col += 1;
curPosz = datalog(:,col);col += 1;
curVelx = datalog(:,col);col += 1;
curVely = datalog(:,col);col += 1;
curVelz = datalog(:,col);col += 1;
curAccx = datalog(:,col);col += 1;
curAccy = datalog(:,col);col += 1;
curAccz = datalog(:,col);col += 1;
if JOINT_OP
desPosJ1= datalog(:,col);col += 1; % ===== des pos joints
curPosJ1= datalog(:,col);col += 1; % ===== cur pos vel tor joints
curPosJ2= datalog(:,col);col += 1;
curPosJ3= datalog(:,col);col += 1;
curPosJ4= datalog(:,col);col += 1;
curPosJ5= datalog(:,col);col += 1;
curPosJ6= datalog(:,col);col += 1;
curPosJ7= datalog(:,col);col += 1;
curVelJ1= datalog(:,col);col += 1;
curVelJ2= datalog(:,col);col += 1;
curVelJ3= datalog(:,col);col += 1;
curVelJ4= datalog(:,col);col += 1;
curVelJ5= datalog(:,col);col += 1;
curVelJ6= datalog(:,col);col += 1;
curVelJ7= datalog(:,col);col += 1;
curTorJ1= datalog(:,col);col += 1;
curTorJ2= datalog(:,col);col += 1;
curTorJ3= datalog(:,col);col += 1;
curTorJ4= datalog(:,col);col += 1;
curTorJ5= datalog(:,col);col += 1;
curTorJ6= datalog(:,col);col += 1;
curTorJ7= datalog(:,col);col += 1;
desTorJ1= datalog(:,col);col += 1;
desTorJ2= datalog(:,col);col += 1;
desTorJ3= datalog(:,col);col += 1;
desTorJ4= datalog(:,col);col += 1;
desTorJ5= datalog(:,col);col += 1;
desTorJ6= datalog(:,col);col += 1;
desTorJ7= datalog(:,col);col += 1;
end

% plotting computation time
figure();

deltaT = t(2:size(t)(1), 1)-t(1:size(t)(1)-1, 1);
plot(t(2:size(t)(1), 1), deltaT);
title("Computing times");
xlabel("Time [sec]");
ylabel("Cycle length [sec]");


% plotting task space tracking
figure();

% plotting position
subplot(display_y, display_x, display_index);display_index += 1;
des = desPosx; cur = curPosx;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
xlabel("Time [sec]");
ylabel("position [m]");
title("x position tracking");

subplot(display_y, display_x, display_index);display_index += 1;
des = desPosy; cur = curPosy;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
xlabel("Time [sec]");
ylabel("position [m]");
title("y position tracking");

subplot(display_y, display_x, display_index);display_index += 1;
des = desPosz; cur = curPosz;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
xlabel("Time [sec]");
ylabel("position [m]");
title("z position tracking");

% plotting velocity
subplot(display_y, display_x, display_index);display_index += 1;
des = desVelx; cur = curVelx;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
title("x velocity tracking");

subplot(display_y, display_x, display_index);display_index += 1;
des = desVely; cur = curVely;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
title("y velocity tracking");

subplot(display_y, display_x, display_index);display_index += 1;
des = desVelz; cur = curVelz;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
title("z velocity tracking");

% plotting acceleration
subplot(display_y, display_x, display_index);display_index += 1;
des = desAccx; cur = curAccx;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
xlabel("Time [sec]");
ylabel("position [m]");
title("x acceleration tracking");

subplot(display_y, display_x, display_index);display_index += 1;
des = desAccy; cur = curAccy;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
xlabel("Time [sec]");
ylabel("position [m]");
title("y acceleration tracking");

subplot(display_y, display_x, display_index);display_index += 1;
des = desAccz; cur = curAccz;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
xlabel("Time [sec]");
ylabel("position [m]");
title("z acceleration tracking");

% ========================= plotting joint values
if JOINT_OP
figure()
display_x = 4;
display_y = 2;
display_index = 1;

subplot(display_y, display_x, display_index);display_index += 1; % 1
des = desTorJ1; cur = curTorJ1;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
xlabel("Time [sec]");
ylabel("torque [N/m]");
title("Joint 1 torques");

subplot(display_y, display_x, display_index);display_index += 1; % 2
des = desTorJ2; cur = curTorJ2;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
xlabel("Time [sec]");
ylabel("torque [N/m]");
title("Joint 2 torques");

subplot(display_y, display_x, display_index);display_index += 1; % 3
des = desTorJ3; cur = curTorJ3;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
xlabel("Time [sec]");
ylabel("torque [N/m]");
title("Joint 3 torques");

subplot(display_y, display_x, display_index);display_index += 1; % 4
des = desTorJ4; cur = curTorJ4;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
xlabel("Time [sec]");
ylabel("torque [N/m]");
title("Joint 4 torques");

subplot(display_y, display_x, display_index);display_index += 1; % 5
des = desTorJ5; cur = curTorJ5;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
xlabel("Time [sec]");
ylabel("torque [N/m]");
title("Joint 5 torques");

subplot(display_y, display_x, display_index);display_index += 1; % 6
des = desTorJ6; cur = curTorJ6;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
xlabel("Time [sec]");
ylabel("torque [N/m]");
title("Joint 6 torques");

subplot(display_y, display_x, display_index);display_index += 1; % 7
des = desTorJ7; cur = curTorJ7;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
xlabel("Time [sec]");
ylabel("torque [N/m]");
title("Joint 7 torques");

% ==================================== joint space tracking
subplot(display_y, display_x, display_index);display_index += 1; % 3
des = desPosJ1; cur = curPosJ1;
plot(t, des, "-b");
hold on;
plot(t, cur, "-r");
hold off;
if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
legend({"desired", "actual"}, 'Location', 'northeast');
else
legend({"desired", "actual"}, 'Location', 'southeast');
endif
xlabel("Time [sec]");
ylabel("torque [N/m]");
title("Base joint position tracking");
end