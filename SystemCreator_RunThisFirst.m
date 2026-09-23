clear all, close all, clc
addpath('./utils');

% Parameters
dt = 0.001;
tspan = dt:dt:10;
x0 = [0; -2.8];

f = @(t,x)([x(2); x(1) - x(1)^3]);
H = @(x)( (1/2)*x(2).^2-(1/2)*x(1).^2 + (1/4)*x(1).^4 );
ode_options = odeset('RelTol',1e-10, 'AbsTol',1e-11);

usesine = 0;
polyorder = 4;
nvar = 2;
%% Trajectory data

[t,y] = ode45(f,tspan,x0,ode_options);

dy = zeros(size(y));
for k=1:length(y)
    dy(k,:) = f(0,y(k,:))';
end

% Construct libraries of candidate functions
Theta = buildTheta(y,nvar,polyorder);
Gamma = buildGamma(y,dy,nvar,polyorder); 

% Compute SVD
[U,S,V] = svd(0*Theta - Gamma,'econ');

% Least-squares Koopman
K = pinv(Theta)*Gamma;
K(abs(K)<1e-12) = 0;
[T,D] = eig(K);
D = diag(D);
[~,IX] = sort(abs(D),'ascend');

% Compute eigenfunction
xi0 = V(:,end);             % from SVD
xi0(abs(xi0)<1e-12) = 0;

xi1 = T(:,IX(1));%+Tls(:,IX(2));  % from least-squares fit
xi1(abs(xi1)<1e-12) = 0; 

% Print coefficients
poolDataLIST({'x','y'},xi0,nvar,polyorder);
poolDataLIST({'x','y'},xi1,nvar,polyorder);


return