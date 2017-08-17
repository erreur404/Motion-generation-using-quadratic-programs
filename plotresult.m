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

% shifting left of 10 seconds
t = t-10;
% limitting the experiment to 60 seconds
index0 = find(t >= 0);
index60 = find (t >= 60);
if (length(index60) == 0)
  index60 = length(t);
end

t = t(index0:index60, 1);

desPosx = datalog(index0:index60,col);col += 1;
desPosy = datalog(index0:index60,col);col += 1;
desPosz = datalog(index0:index60,col);col += 1;
desVelx = datalog(index0:index60,col);col += 1;
desVely = datalog(index0:index60,col);col += 1;
desVelz = datalog(index0:index60,col);col += 1;
desAccx = datalog(index0:index60,col);col += 1;
desAccy = datalog(index0:index60,col);col += 1;
desAccz = datalog(index0:index60,col);col += 1;
curPosx = datalog(index0:index60,col);col += 1;
curPosy = datalog(index0:index60,col);col += 1;
curPosz = datalog(index0:index60,col);col += 1;
curVelx = datalog(index0:index60,col);col += 1;
curVely = datalog(index0:index60,col);col += 1;
curVelz = datalog(index0:index60,col);col += 1;
curAccx = datalog(index0:index60,col);col += 1;
curAccy = datalog(index0:index60,col);col += 1;
curAccz = datalog(index0:index60,col);col += 1;
if JOINT_OP
desPosJ1= datalog(index0:index60,col);col += 1; % ===== des pos joints
curPosJ1= datalog(index0:index60,col);col += 1; % ===== cur pos vel tor joints
curPosJ2= datalog(index0:index60,col);col += 1;
curPosJ3= datalog(index0:index60,col);col += 1;
curPosJ4= datalog(index0:index60,col);col += 1;
curPosJ5= datalog(index0:index60,col);col += 1;
curPosJ6= datalog(index0:index60,col);col += 1;
curPosJ7= datalog(index0:index60,col);col += 1;
curVelJ1= datalog(index0:index60,col);col += 1;
curVelJ2= datalog(index0:index60,col);col += 1;
curVelJ3= datalog(index0:index60,col);col += 1;
curVelJ4= datalog(index0:index60,col);col += 1;
curVelJ5= datalog(index0:index60,col);col += 1;
curVelJ6= datalog(index0:index60,col);col += 1;
curVelJ7= datalog(index0:index60,col);col += 1;

% computing real time acceleration from velocities
col -= 7; % going back to velocities
echant = round(length(t)/60);
curAccJ1= datalog(index0+echant:index60+echant,col)-datalog(index0:index60,col);col += 1;
curAccJ2= datalog(index0+echant:index60+echant,col)-datalog(index0:index60,col);col += 1;
curAccJ3= datalog(index0+echant:index60+echant,col)-datalog(index0:index60,col);col += 1;
curAccJ4= datalog(index0+echant:index60+echant,col)-datalog(index0:index60,col);col += 1;
curAccJ5= datalog(index0+echant:index60+echant,col)-datalog(index0:index60,col);col += 1;
curAccJ6= datalog(index0+echant:index60+echant,col)-datalog(index0:index60,col);col += 1;
curAccJ7= datalog(index0+echant:index60+echant,col)-datalog(index0:index60,col);col += 1;

curTorJ1= datalog(index0:index60,col);col += 1;
curTorJ2= datalog(index0:index60,col);col += 1;
curTorJ3= datalog(index0:index60,col);col += 1;
curTorJ4= datalog(index0:index60,col);col += 1;
curTorJ5= datalog(index0:index60,col);col += 1;
curTorJ6= datalog(index0:index60,col);col += 1;
curTorJ7= datalog(index0:index60,col);col += 1;
desTorJ1= datalog(index0:index60,col);col += 1;
desTorJ2= datalog(index0:index60,col);col += 1;
desTorJ3= datalog(index0:index60,col);col += 1;
desTorJ4= datalog(index0:index60,col);col += 1;
desTorJ5= datalog(index0:index60,col);col += 1;
desTorJ6= datalog(index0:index60,col);col += 1;
desTorJ7= datalog(index0:index60,col);col += 1;

out_jointPosLimitInfJ1 = datalog(index0:index60,col);col += 1;
out_jointPosLimitInfJ2 = datalog(index0:index60,col);col += 1;
out_jointPosLimitInfJ3 = datalog(index0:index60,col);col += 1;
out_jointPosLimitInfJ4 = datalog(index0:index60,col);col += 1;
out_jointPosLimitInfJ5 = datalog(index0:index60,col);col += 1;
out_jointPosLimitInfJ6 = datalog(index0:index60,col);col += 1;
out_jointPosLimitInfJ7 = datalog(index0:index60,col);col += 1;

out_jointPosLimitSupJ1 = datalog(index0:index60,col);col += 1;
out_jointPosLimitSupJ2 = datalog(index0:index60,col);col += 1;
out_jointPosLimitSupJ3 = datalog(index0:index60,col);col += 1;
out_jointPosLimitSupJ4 = datalog(index0:index60,col);col += 1;
out_jointPosLimitSupJ5 = datalog(index0:index60,col);col += 1;
out_jointPosLimitSupJ6 = datalog(index0:index60,col);col += 1;
out_jointPosLimitSupJ7 = datalog(index0:index60,col);col += 1;

out_jointVelLimitInfJ1 = datalog(index0:index60,col);col += 1;
out_jointVelLimitInfJ2 = datalog(index0:index60,col);col += 1;
out_jointVelLimitInfJ3 = datalog(index0:index60,col);col += 1;
out_jointVelLimitInfJ4 = datalog(index0:index60,col);col += 1;
out_jointVelLimitInfJ5 = datalog(index0:index60,col);col += 1;
out_jointVelLimitInfJ6 = datalog(index0:index60,col);col += 1;
out_jointVelLimitInfJ7 = datalog(index0:index60,col);col += 1;

out_jointVelLimitSupJ1 = datalog(index0:index60,col);col += 1;
out_jointVelLimitSupJ2 = datalog(index0:index60,col);col += 1;
out_jointVelLimitSupJ3 = datalog(index0:index60,col);col += 1;
out_jointVelLimitSupJ4 = datalog(index0:index60,col);col += 1;
out_jointVelLimitSupJ5 = datalog(index0:index60,col);col += 1;
out_jointVelLimitSupJ6 = datalog(index0:index60,col);col += 1;
out_jointVelLimitSupJ7 = datalog(index0:index60,col);col += 1;

out_jointAccLimitInfJ1 = datalog(index0:index60,col);col += 1;
out_jointAccLimitInfJ2 = datalog(index0:index60,col);col += 1;
out_jointAccLimitInfJ3 = datalog(index0:index60,col);col += 1;
out_jointAccLimitInfJ4 = datalog(index0:index60,col);col += 1;
out_jointAccLimitInfJ5 = datalog(index0:index60,col);col += 1;
out_jointAccLimitInfJ6 = datalog(index0:index60,col);col += 1;
out_jointAccLimitInfJ7 = datalog(index0:index60,col);col += 1;

out_jointAccLimitSupJ1 = datalog(index0:index60,col);col += 1;
out_jointAccLimitSupJ2 = datalog(index0:index60,col);col += 1;
out_jointAccLimitSupJ3 = datalog(index0:index60,col);col += 1;
out_jointAccLimitSupJ4 = datalog(index0:index60,col);col += 1;
out_jointAccLimitSupJ5 = datalog(index0:index60,col);col += 1;
out_jointAccLimitSupJ6 = datalog(index0:index60,col);col += 1;
out_jointAccLimitSupJ7 = datalog(index0:index60,col);col += 1;

out_jointAccDynLimitInfJ1 = datalog(index0:index60,col);col += 1;
out_jointAccDynLimitInfJ2 = datalog(index0:index60,col);col += 1;
out_jointAccDynLimitInfJ3 = datalog(index0:index60,col);col += 1;
out_jointAccDynLimitInfJ4 = datalog(index0:index60,col);col += 1;
out_jointAccDynLimitInfJ5 = datalog(index0:index60,col);col += 1;
out_jointAccDynLimitInfJ6 = datalog(index0:index60,col);col += 1;
out_jointAccDynLimitInfJ7 = datalog(index0:index60,col);col += 1;

out_jointAccDynLimitSupJ1 = datalog(index0:index60,col);col += 1;
out_jointAccDynLimitSupJ2 = datalog(index0:index60,col);col += 1;
out_jointAccDynLimitSupJ3 = datalog(index0:index60,col);col += 1;
out_jointAccDynLimitSupJ4 = datalog(index0:index60,col);col += 1;
out_jointAccDynLimitSupJ5 = datalog(index0:index60,col);col += 1;
out_jointAccDynLimitSupJ6 = datalog(index0:index60,col);col += 1;
out_jointAccDynLimitSupJ7 = datalog(index0:index60,col);col += 1;

out_jointTorqueLimitInfJ1 = datalog(index0:index60,col);col += 1;
out_jointTorqueLimitInfJ2 = datalog(index0:index60,col);col += 1;
out_jointTorqueLimitInfJ3 = datalog(index0:index60,col);col += 1;
out_jointTorqueLimitInfJ4 = datalog(index0:index60,col);col += 1;
out_jointTorqueLimitInfJ5 = datalog(index0:index60,col);col += 1;
out_jointTorqueLimitInfJ6 = datalog(index0:index60,col);col += 1;
out_jointTorqueLimitInfJ7 = datalog(index0:index60,col);col += 1;

out_jointTorqueLimitSupJ1 = datalog(index0:index60,col);col += 1;
out_jointTorqueLimitSupJ2 = datalog(index0:index60,col);col += 1;
out_jointTorqueLimitSupJ3 = datalog(index0:index60,col);col += 1;
out_jointTorqueLimitSupJ4 = datalog(index0:index60,col);col += 1;
out_jointTorqueLimitSupJ5 = datalog(index0:index60,col);col += 1;
out_jointTorqueLimitSupJ6 = datalog(index0:index60,col);col += 1;
out_jointTorqueLimitSupJ7 = datalog(index0:index60,col);col += 1;

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
display_y = 7;
display_index = 1;

% plotting torque values
for joint=1:display_y
    %sprintf("displaying torque graph")
    subplot(display_y, display_x, display_index);display_index += 1; % 1
    des = eval(sprintf("desTorJ%d",joint)); cur = eval(sprintf("curTorJ%d", joint));
    sup = eval(sprintf("out_jointTorqueLimitSupJ%d", joint)); inf = eval(sprintf("out_jointTorqueLimitInfJ%d", joint));
    plot(t, des, "-b");
    hold on;
    plot(t, cur, "-r");
    hold on;
    plot(t, inf, "-y");
    hold on;
    plot(t, sup, "-k");
    hold off;
    if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
        %legend({"desired", "actual", "inf limit", "sup limit"}, 'Location', 'northeast');
    else
        %legend({"desired", "actual", "inf limit", "sup limit"}, 'Location', 'southeast');
    endif
    xlabel("Time [sec]");
    ylabel("torque [N/m]");
    title(sprintf("Joint %d torques", joint));

    %sprintf("displaying accel graph") % no accel feedback ?
    subplot(display_y, display_x, display_index);display_index += 1; % 1
    infStat = eval(sprintf("out_jointAccLimitInfJ%d",joint)); cur = eval(sprintf("curAccJ%d", joint)); supStat = eval(sprintf("out_jointAccLimitSupJ%d", joint));
    inf = eval(sprintf("out_jointAccDynLimitInfJ%d", joint)); sup = eval(sprintf("out_jointAccDynLimitSupJ%d", joint));
    plot(t, infStat, "-b");
    hold on;
    plot(t, supStat, "-c");
    hold on;
    plot(t, cur, "-r");
    hold on;
    plot(t, inf, "-y");
    hold on;
    plot(t, sup, "-k");
    hold off;
    if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
        %legend({"actual", "static inf limit", "static sup limit", "dyn inf limit", "dyn sup limit"}, 'Location', 'northeast');
    else
        %legend({"actual", static inf limit", "static sup limit", "dyn inf limit", "dyn sup limit"}, 'Location', 'northeast');
    endif
    xlabel("Time [sec]");
    ylabel("acceleration [m/s²]");
    title(sprintf("Joint %d acceleration", joint));

    %sprintf("displaying velocity graph")
    subplot(display_y, display_x, display_index);display_index += 1; % 1
    cur = eval(sprintf("curVelJ%d", joint));
    sup = eval(sprintf("out_jointVelLimitSupJ%d", joint)); inf = eval(sprintf("out_jointVelLimitInfJ%d", joint));
    plot(t, cur, "-r");
    hold on;
    plot(t, inf, "-y");
    hold on;
    plot(t, sup, "-k");
    hold off;
    if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
        %legend({"actual", "inf limit", "sup limit"}, 'Location', 'northeast');
    else
        %legend({"actual", "inf limit", "sup limit"}, 'Location', 'southeast');
    endif
    xlabel("Time [sec]");
    ylabel("speed [rad/s]");
    title(sprintf("Joint %d speeds", joint));


    %sprintf("displaying position graph")
    subplot(display_y, display_x, display_index);display_index += 1; % 1
    cur = eval(sprintf("curPosJ%d", joint));
    sup = eval(sprintf("out_jointPosLimitSupJ%d", joint)); inf = eval(sprintf("out_jointPosLimitInfJ%d", joint));
    plot(t, cur, "-r");
    hold on;
    plot(t, inf, "-y");
    hold on;
    plot(t, sup, "-k");
    hold off;
    if (mean(cur(floor(size(cur)(1)/2):size(cur)(1))) < (max(cur) + min(cur))/2)
        %legend({"actual", "inf limit", "sup limit"}, 'Location', 'northeast');
    else
        %legend({"actual", "inf limit", "sup limit"}, 'Location', 'southeast');
    endif
    xlabel("Time [sec]");
    ylabel("position [rad]");
    title(sprintf("Joint %d positions", joint));

    %sprintf("                    for joint %d", joint)
end

% ==================================== joint space tracking

figure();
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
ylabel("position (rad)");
title("Base joint position tracking");
end
