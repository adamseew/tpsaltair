title('Quadrotor 1d plot')

subplot(3, 1, 1);
D=dlmread('.tmp_00.dat');
plot(D(:,1))
ylabel('x(t)')
xlabel('t')
title('Distance on t')

subplot(3, 1, 2);
plot(D(:,3))
ylabel('px(t)')
xlabel('t')
title('Momentum on t')

subplot(3, 1, 3);
plot(D(:,2))
ylabel('\theta(t)')
xlabel('t')
title('Pitch angle')