function phi = getHyperParameters(X,Y)
%function phi = getHyperParameters(X,Y)
% Estimate hyperparameters (in exponential Kernel) based on p visited cells
%   X is p-by-2 matrix of spatial horizontal/vertical coordinates
%   Y is p-by-1 vector of (scalar) observations
%
%   phi is 2-by-1 vector with k(i,j) = phi(2)^2 * exp(-||iPos-jPos||/phi(1))

KREIDL = 1; % Toggles between MATLAB's fitrgp command or Kreidl's method

if KREIDL % Kreidl's method
  phi = fmincon(@(x) GPNegLogLikelihood(x,[X Y]),...
                [mean(std(X));std(Y)/sqrt(2)],[],[],[],[],...
                [0; 0],[Inf; Inf]);
else % MATLAB's method
  RGP = fitrgp(X,Y,'KernelFunction','exponential');
  phi = RGP.KernelInformation.KernelParameters;
end

end

function NLL = GPNegLogLikelihood(phi,tbl)

xC = tbl(:,1); yC = tbl(:,2); z = tbl(:,3);

p = length(z); K = eye(p)*phi(2)^2;
for i = 1:p
  iPos = [xC(i); yC(i)]; % location of point i
  for j = i+1:p
    jPos = [xC(j); yC(j)]; % location of point j
    K(i,j) = phi(2)^2*exp(-norm(iPos-jPos)/phi(1)); K(j,i) = K(i,j);
  end
end

NLL = 0.5*(z'*mldivide(K,z) + log(det(K)) + p*log(2*pi)); 

end