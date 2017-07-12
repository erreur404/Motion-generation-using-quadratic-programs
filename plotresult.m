
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
desPosJ7 = datalog(:,col);col += 1;
curPosJ7 = datalog(:,col);col += 1;

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
