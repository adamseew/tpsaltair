title('Program output plot')
xlabel('x')
ylabel('y')

subplot(1, 2, 1);
title('Shoots needed to obtain the solution')
hold on

files = dir('*.dat');
L = length(files);
for k = 1:L
    D=dlmread(files(k).name);
    plot(D(:,1),D(:,2))
    legendInfo{k} = [' № ' , num2str(k)];
end
legend(legendInfo)

set(gca,'ylim',[0 340]);
set(gca,'xlim',[0 980]);

subplot(1, 2, 2);
title('Winning shoot with a tollerance \epsilon')
hold on

plot(D(:,1),D(:,2))

set(gca,'ylim',[0 340]);
set(gca,'xlim',[0 980]);