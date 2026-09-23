addpath('./utils');
path2figs = './../Figures/DUFFING/'; mkdir(path2figs)
ModelName = 'DiscoverDuffing_KRONICvsEDMDc';
close all
% Parameters
dt = 0.01;
tspan = dt:dt:50;
x0 = [0; -2.3];

% Function definitions
f = @(t,x)([x(2); x(1) - x(1)^3]);
H = @(x)( (1/2)*x(2).^2-(1/2)*x(1).^2 + (1/4)*x(1).^4 );
ode_options = odeset('RelTol',1e-10, 'AbsTol',1e-11);
nvar = 2;

%% Collect trajectory data
% Unforced
[t,y] = ode45(f,tspan,x0,ode_options);

% Forced
B = [0; 1];
forcing = @(x,t) [(2*(sin(2*pi*.1*t).*sin(2*pi*1*t)))];
f = @(t,x) [x(2); x(1) - x(1)^3]  + B*forcing(x,t); 
H = @(x)( (1/2)*x(2).^2-(1/2)*x(1).^2 + (1/4)*x(1).^4 );

[tF,yF] = ode45(f,tspan,x0,ode_options);

uF = forcing(yF,tspan)';
U = uF';

figure; hold on, box on
plot3(y(:,1),y(:,2),zeros(size(y(:,2))),'-','Color',[0,0,0.7],'LineWidth',2)
color_line3(yF(:,1),yF(:,2),zeros(size(yF(:,2))),uF,'LineWidth',2);%,'--','Color','r','LineWidth',2)
xlabel('x1'), ylabel('x2')

figure,plot(uF)
%% KRONIC
% Parameters
pKRONIC.phi = V(:,end);
pKRONIC.polyorder = 4;
pKRONIC.usesine = 0;
% Compute derivatives using fourth order central difference
dy = zeros(length(y)-5,nvar);
for i=3:length(y)-3
    for k=1:nvar
        dy(i-2,k) = (1/(12*dt))*(-y(i+2,k)+8*y(i+1,k)-8*y(i-1,k)+y(i-2,k));
    end
end
x = y(3:end-3,1:nvar);
dx = dy;
tx = t(3:end-3);

% Construct libraries
Theta = buildTheta(x,nvar,pKRONIC.polyorder);
Gamma = buildGamma(x,dx,nvar,pKRONIC.polyorder);

% Compute SVD
[U,S,V] = svd(0*Theta - Gamma,'econ');
KRONIC.phi = V(:,end);

% Remove values below small threshold
KRONIC.phi(abs(KRONIC.phi)<1e-6) = 0;

% Print coefficients
poolDataLIST({'x','y'},KRONIC.phi,nvar,pKRONIC.polyorder);
%% Control parameters & functions 
x_REF = [0; 0];
H_REF = H(x_REF);

f = @(t,x,u)([x(2); x(1)-x(1)^3] + B*u);
Hc = @(x)((1/2)*x(2).^2-(x(1).^2)/2 + (1/4)*x(1).^4 - H_REF);
gradH = @(x)([-x(1) + x(1)^3; x(2)]);


Q = 5*eye(2);                   % x weights
Ru = 1;                         % u weights
R = 0;                          % du weights
QH = 5;                         % phi(=H)  weights
%% KRONIC (analytic)
p_analyticKRONIC.H = H;


% LQR
tic
gain = @(x)(lqr(0,(gradH(x)'*B),QH,Ru));
[tHistory_analyticKRONIC_LQR,xHistory_analyticKRONIC_LQR] = ode45(@(t,x)f(t,x,-gain(x)*Hc(x)),tspan,x0);
xHistory_analyticKRONIC_LQR = xHistory_analyticKRONIC_LQR';
uHistory_analyticKRONIC_LQR = zeros(size(tHistory_analyticKRONIC_LQR));
for i = 1:length(xHistory_analyticKRONIC_LQR)
    uHistory_analyticKRONIC_LQR(i) = -gain(xHistory_analyticKRONIC_LQR(:,i))*Hc(xHistory_analyticKRONIC_LQR(:,i));
end
toc
%% KRONIC (data)

% LQR
tic
pKRONIC.B = @(x) [(buildThetaGradient(x',1,nvar,pKRONIC.polyorder)*pKRONIC.phi);(buildThetaGradient(x',2,nvar,pKRONIC.polyorder,pKRONIC.usesine)*pKRONIC.phi)]'*B;
pKRONIC.H = @(x) buildTheta(x',nvar,pKRONIC.polyorder)*pKRONIC.phi;

pKRONIC.Hc = @(x) [buildTheta(x',nvar,pKRONIC.polyorder)*KRONIC.phi - pKRONIC.H(x_REF)];   

gain = @(x)(lqr(0,pKRONIC.B(x),QH,Ru));
[tHistory_KRONIC_LQR,xHistory_KRONIC_LQR] = ode45(@(t,x)f(t,x,-gain(x)*pKRONIC.Hc(x)),tspan,x0);
xHistory_KRONIC_LQR = xHistory_KRONIC_LQR';
uHistory_KRONIC_LQR = zeros(size(tHistory_KRONIC_LQR));
for i = 1:length(xHistory_KRONIC_LQR)
    uHistory_KRONIC_LQR(i) = -gain(xHistory_KRONIC_LQR(:,i))*pKRONIC.Hc(xHistory_KRONIC_LQR(:,i));
end
toc
%%
wq=y(:,1);

figure; hold on, box on
plot(y(:,1),y(:,2),'-','Color',[0 0 0]+0.05*8,'LineWidth',1.3)
plot([wq;wq],[+sqrt(wq.^2-0.5*wq.^4);-sqrt(wq.^2-0.5*wq.^4)],'-','Color','g','LineWidth',2)

%plot(wq,[sqrt(wq.^2-0.5*wq.^4),-sqrt(wq.^2-0.5*wq.^4)]','-','Color','g','LineWidth',2)
%plot(wq,-+sqrt(wq.^2-0.5*wq.^4),'-','Color','g','LineWidth',2)
plot(xHistory_analyticKRONIC_LQR(1,:),xHistory_analyticKRONIC_LQR(2,:),'-','Color','r','LineWidth',1)
plot(xHistory_KRONIC_LQR(1,:),xHistory_KRONIC_LQR(2,:),'--','Color','m','LineWidth',1)
ylabel('x2'), xlabel('x1')
legend('Initial Orbit','Desired Orbit','Analytically controlled system trajectory','Model-free control trajcetot')
%% Show Actuation
figure
hold on
plot(tHistory_analyticKRONIC_LQR,uHistory_analyticKRONIC_LQR,'-r','LineWidth',2),
plot(tHistory_KRONIC_LQR,uHistory_KRONIC_LQR,'--m','LineWidth',2),
xlabel('time'),ylabel('u')
set(gca,'FontSize',16)
set(gcf,'Position',[100 100 600 400])
set(gcf,'PaperPositionMode','auto')
Nt = length(tHistory_KRONIC_LQR);

%% Show cumulative cost in terms of state
LineWidth = 3;
[JvalsKRONIC] = evalCostFun(xHistory_KRONIC_LQR,uHistory_KRONIC_LQR',Q,Ru,x_REF);

%% Compare to linearized LQR
A = [0 1;1 0];
B = [0;1];
C = lqr(A,B,Q,3);
vf = @(t,x,u) A*x +  [0; - x(1)^3]  + B*u; 

[t,xLQR] = ode45(@(t,x)f(t,x,-C*x),tspan,x0);
figure,plot(t,xLQR,'--')
hold on
plot(tHistory_KRONIC_LQR,xHistory_KRONIC_LQR,'LineWidth',2)
ylabel('x_i'), xlabel('t')
legend('x1 for KOOQR','x2 for KOOQR','x1 for LQR','x2 for LQR')

JLQR = cumsum(xLQR(:,1).^2 + xLQR(:,2).^2 + (C*xLQR')'.^2)';
figure, plot(tspan,cumsum(JvalsKRONIC),'-b','LineWidth',2)
hold on
plot(tspan,JLQR,'k','LineWidth',1.2);
legend('Cost function for KOOQR','Cost function for LQR')
ylabel('J'), xlabel('t')
