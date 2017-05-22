function [ output_args ] = plotbc3( a,scale,y )
s=linspace(0,1,50);
s=s*scale;
for j=1:length(s)
    b(j)=bezier(a,s(j));
end
y=y*ones(1,length(s));
plot3(s,y,b)
end

