title('Program output plot')
xlabel('x')
ylabel('y')

files = dir('*.dat');
L = length(files);

figure 
hold on

for k = 1:L

    D=dlmread(files(k).name);
    plot(D(:,1),D(:,2))
    % fwbw
    % plot(D(:,1))
    legendInfo{k} = [files(k).name];

    hold on
end

legend(legendInfo)