title('Quadrotor 1d plot')

hold on

files = dir('*.dat');
L = length(files);

subplot(3, 2, 1);
for k = 1:L

    D=dlmread(files(k).name);
    plot(D(:,1))
    
    hold on
end

x1 = [0, 160, 160, 0, 0];
y1 = [0.52, 0.52, 0.95, 0.95, 0.52];
plot(x1, y1, 'r-', 'LineWidth', 1)

x1 = [196, 300, 300, 196, 196];
y1 = [0.52, 0.52, 0.95, 0.95, 0.52];
plot(x1, y1, 'r-', 'LineWidth', 1)

ylabel('x(t)')
xlabel('t')
title('Distance on t')

set(gca,'ylim',[0 1.2]);
set(gca,'xlim',[0 300]);

subplot(3, 2, 3);
for k = 1:L

    D=dlmread(files(k).name);
    plot(D(:,3))

    hold on

end

ylabel('px(t)')
xlabel('t')
title('Momentum on t')

set(gca,'ylim',[0 60]);
set(gca,'xlim',[0 300]);

subplot(3, 2, 5);
for k = 1:L

    D=dlmread(files(k).name);
    plot(D(:,2))

    hold on

end

ylabel('\theta(t)')
xlabel('t')
title('Pitch angle')

set(gca,'ylim',[-0.6 0]);
set(gca,'xlim',[0 300]);

subplot(3, 2, 2);

D=dlmread('.tmp_107.dat');
plot(D(:,1))

hold on

D=dlmread('.tmp_117.dat');
plot(D(:,1))
D=dlmread('.tmp_127.dat');
plot(D(:,1))

x1 = [0, 160, 160, 0, 0];
y1 = [0.52, 0.52, 0.95, 0.95, 0.52];
plot(x1, y1, 'r-', 'LineWidth', 1)

x1 = [196, 300, 300, 196, 196];
y1 = [0.52, 0.52, 0.95, 0.95, 0.52];
plot(x1, y1, 'r-', 'LineWidth', 1)

ylabel('x(t)')
xlabel('t')
title('Distance on t')

set(gca,'ylim',[0 1.2]);
set(gca,'xlim',[0 300]);

subplot(3, 2, 4);
D=dlmread('.tmp_107.dat');
plot(D(:,3))

hold on

D=dlmread('.tmp_117.dat');
plot(D(:,3))
D=dlmread('.tmp_127.dat');
plot(D(:,3))

ylabel('px(t)')
xlabel('t')
title('Momentum on t')

set(gca,'ylim',[0 60]);
set(gca,'xlim',[0 300]);

subplot(3, 2, 6);
D=dlmread('.tmp_107.dat');
plot(D(:,2))

hold on

D=dlmread('.tmp_117.dat');
plot(D(:,2))
D=dlmread('.tmp_127.dat');
plot(D(:,2))

ylabel('\theta(t)')
xlabel('t')
title('Pitch angle')

set(gca,'ylim',[-0.6 0]);
set(gca,'xlim',[0 300]);