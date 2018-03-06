title('Program output plot')
xlabel('t [s]')
ylabel('y [m]')
hold on

files = dir('*.dat');
L = length(files);
for k = 1:L
    %strcmp(files(k).name, '.dat')
    D=dlmread(files(k).name);
    plot(D(:,1),D(:,2))
end

set(gca,'ylim',[0 120]);