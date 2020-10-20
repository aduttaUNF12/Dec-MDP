function [xHat,Khat] = posteriorGP(GP,Y,Kv)
% function [xHat,Khat] = posteriorGP(GP,Y,Kv)

  I = Y(:,1); Tau = mrdivide(GP.Sigma(:,I),GP.Sigma(I,I)+Kv);
  Khat = GP.Sigma - Tau*GP.Sigma(I,:);
  xHat = GP.Mu + Tau*(Y(:,2)-GP.Mu(I));
