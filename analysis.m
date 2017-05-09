% See the figure for a
figure
a=[];
for i = 1:length(outputs)
    a=outputs{i}.a(1,:);
    subplot(3,1,i)
    plot(1:24,a)
end