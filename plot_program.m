title('Program output plot')
xlabel('x')
ylabel('y')

subplot(1, 2, 1);
hold on

files = dir('*.dat');
L = length(files);
for k = 1:L
    D=dlmread(files(k).name);
    plot(D(:,1),D(:,2))
end

set(gca,'ylim',[0 120]);

subplot(1, 2, 2);

D=dlmread(".tmp_28.dat");
plot(D(:,1),D(:,2))

set(gca,'ylim',[0 120]);