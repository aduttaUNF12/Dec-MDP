%% api to update mdl
% Xtrain and YTrain are supersets of the previous training data.
function newmdl = update_GP_model(mdl,Xtrain, Ytrain) % a very simple updating api, assumes defaults, will need to be modified for nondefaults
    kernelparams = mdl.KernelInformation.KernelParameters;
    inisigma = mdl.Sigma;
    beta = mdl.Beta;
    newmdl = fitrgp(Xtrain, Ytrain, 'FitMethod', 'exact','KernelFunction','squaredexponential', 'Sigma', inisigma, 'Beta', beta, 'KernelParameters', kernelparams);
end