title('Program output plot')
xlabel('x')
ylabel('y')

files = dir('*.dat');
L = length(files);

figure 
hold on

for k = 1:L

    D=dlmread(files(k).name);
    plot(D(:,1))
    if mod(k,2) == 0
        legendInfo{k} = ['forward'];
    else
        legendInfo{k} = ['backward'];
    end

    hold on
end

legend(legendInfo)