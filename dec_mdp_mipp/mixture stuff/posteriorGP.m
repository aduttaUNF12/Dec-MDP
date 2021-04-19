function [xHat,Khat] = posteriorGP(GP,Y,Kv)
% function [xHat,Khat] = posteriorGP(GP,Y,Kv)
   
  I = Y(:,1); 
  %disp(size(GP.Sigma(:,I)));
  %disp(size(GP.Sigma(I,I)+Kv));
  warning('off')
  Tau = mrdivide(GP.Sigma(:,I),GP.Sigma(I,I)+Kv);
  %disp(size(Tau));
  %disp(size(GP.Sigma(I,:)));
  Khat = GP.Sigma - Tau*GP.Sigma(I,:);
  xHat = GP.Mu + Tau*(Y(:,2)-GP.Mu(I));
