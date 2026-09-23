function metrics = audit_koopman(sourceDir, outputDir)
% Numerical review added in September 2026; original project files are unchanged.
% From the repository root: addpath('tools'); metrics = audit_koopman;
% This creates docs/validation-results.json and does not execute ControllerDesign.
repoRoot = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1
    sourceDir = repoRoot;
end
if nargin < 2
    outputDir = fullfile(repoRoot, 'docs');
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end
originalPath = path;
restorePath = onCleanup(@() path(originalPath));
addpath(fullfile(sourceDir, 'utils'));
f = @(t,x) [x(2); x(1)-x(1)^3];
options = odeset('RelTol',1e-10,'AbsTol',1e-11);
Hcoeff = zeros(14,1);
Hcoeff([3 5 10]) = [-0.5 0.5 0.25];
unitH = Hcoeff / norm(Hcoeff);

[t,y] = ode45(f,0.001:0.001:10,[0;-2.8],options);
dy = [y(:,2),y(:,1)-y(:,1).^3];
Theta = buildTheta(y,2,4);
Gamma = buildGamma(y,dy,2,4);
[~,S,V] = svd(-Gamma,'econ');
phiExact = V(:,end);
K = pinv(Theta)*Gamma;
K(abs(K)<1e-12) = 0;
[T,D] = eig(K);
[~,idx] = sort(abs(diag(D)),'ascend');
metrics.matlabVersion = version;
metrics.discoverySamples = size(y,1);
metrics.librarySize = size(Theta,2);
metrics.exactCoefficients = phiExact';
metrics.exactSignAlignedCoefficientError = norm(sign(phiExact'*unitH)*phiExact-unitH);
metrics.exactSmallestSingularValues = diag(S(end-2:end,end-2:end))';
metrics.exactRmsDerivativeResidual = norm(Gamma*phiExact)/sqrt(size(Gamma,1));
metrics.leastSquaresSmallestEigenvalue = D(idx(1),idx(1));
phiLS = T(:,idx(1));
metrics.leastSquaresSignAlignedCoefficientError = min(norm(phiLS-unitH),norm(phiLS+unitH));

[t,y] = ode45(f,0.01:0.01:50,[0;-2.3],options);
dy = (-y(5:end-1,:)+8*y(4:end-2,:)-8*y(2:end-4,:)+y(1:end-5,:))/(12*0.01);
x = y(3:end-3,:);
Gamma = buildGamma(x,dy,2,4);
[~,S,V] = svd(-Gamma,'econ');
phiData = V(:,end);
phiData(abs(phiData)<1e-6) = 0;
metrics.controllerSamples = size(y,1);
metrics.derivativeSamples = size(x,1);
metrics.dataCoefficients = phiData';
metrics.dataSignAlignedCoefficientError = min(norm(phiData-unitH),norm(phiData+unitH));
metrics.dataRmsDerivativeResidual = norm(Gamma*phiData)/sqrt(size(Gamma,1));
metrics.dataSmallestSingularValues = diag(S(end-2:end,end-2:end))';
metrics.coefficientSetDotProduct = phiData'*phiExact;
metrics.dataToExactPhysicalScale = (phiData'*Hcoeff)/(Hcoeff'*Hcoeff);
metrics.outOfDomainSeparatrixSamples = sum(y(:,1).^2-0.5*y(:,1).^4<0);

points = [0.3 0.7; -1.1 0.4; 0 0; 1.3 -0.9];
velocities = [0.7 0.3-0.3^3; 0.4 -1.1-(-1.1)^3; 0 0; -0.9 1.3-1.3^3];
h = 1e-6;
metrics.gradientRelativeErrors = zeros(4,2);
metrics.gammaRelativeErrors = zeros(1,5);
for order = 1:5
    finiteGamma = (buildTheta(points+h*velocities,2,order)-buildTheta(points-h*velocities,2,order))/(2*h);
    exactGamma = buildGamma(points,velocities,2,order);
    metrics.gammaRelativeErrors(order) = norm(exactGamma-finiteGamma,'fro')/max(1,norm(exactGamma,'fro'));
    if order <= 4
        for j = 1:2
            plus = points; minus = points;
            plus(:,j) = plus(:,j)+h; minus(:,j) = minus(:,j)-h;
            numerical = (buildTheta(plus,2,order)-buildTheta(minus,2,order))/(2*h);
            analytical = buildThetaGradient(points,j,2,order,0);
            metrics.gradientRelativeErrors(order,j) = norm(numerical-analytical,'fro')/max(1,norm(analytical,'fro'));
        end
    end
end
metrics.gradientOrderFiveColumns = size(buildThetaGradient(points,1,2,5,0),2);
metrics.thetaOrderFiveColumns = size(buildTheta(points,2,5),2);
try
    lqr(0,0,5,1);
    metrics.zeroInputGainBehavior = 'Returned without an error';
catch exception
    metrics.zeroInputGainBehavior = exception.message;
end
metrics.colorLine3OnPath = which('color_line3');
metrics.evalCostFunOnPath = which('evalCostFun');
metrics.initialAnalyticalInput = -lqr(0,-2.3,5,1)*(0.5*2.3^2);
metrics.initialMixedCoefficientInput = -lqr(0,buildThetaGradient([0 -2.3],2,2,4,0)*phiExact,5,1)*(buildTheta([0 -2.3],2,4)*phiData);
metrics.initialConsistentCoefficientInput = -lqr(0,buildThetaGradient([0 -2.3],2,2,4,0)*phiData,5,1)*(buildTheta([0 -2.3],2,4)*phiData);
fid = fopen(fullfile(outputDir,'validation-results.json'),'w');
cleanupFile = onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',jsonencode(metrics,PrettyPrint=true));
disp(jsonencode(metrics,PrettyPrint=true));
end
