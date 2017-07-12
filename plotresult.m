
display_x = 3;
display_y = 1;
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
subplot(display_y, display_x, display_index);display_index += 1;
plot(t, desPosx, "-b");
hold on;
plot(t, curPosx, "-r");
hold off;
legend("desired", "actual");
xlabel("Time [sec]");
ylabel("position [m]");
title("X position tracking");
