title('Program output plot')
xlabel('x')
ylabel('y')
hold on

files = dir('*.dat');
L = length(files);
for k = 1:L
    D=dlmread(files(k).name);
    plot(D(:,1),D(:,2))
end

set(gca,'ylim',[0 120]);