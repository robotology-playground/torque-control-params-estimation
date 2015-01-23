function a = lineRegress( x, y)%, option )
%LINEREGRESS Summary of this function goes here
%   Detailed explanation goes here
N = size(x,1);
X = [x ones(N,1)]; % Add column of 1's to include constant term in regression
a = regress(y,X);   % = [a1; a0]
%plot(x,X*a,option);  % This line perfectly overlays the previous fit line


end

