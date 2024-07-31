clc; clear; close all;

% Particle filter localization
%clear;clc;
v = VideoWriter("Particle Localization.avi");
open(v);

% Time
Tf = 3.8;
dt = 0.1;
T = 0:dt:Tf;

% Initial State
x0 = [0 0 0]';
vx = -5;
vy = 2;

% Control inputs
u = 5*ones(2, length(T));
u(2,:)=0.3/5*u(2,:);

% Disturbance model
if ~exist('R','var')
    R = [0.000005 0 0 0 0; 
         0 0.000005 0 0 0; 
         0 0 0.000005 0 0;
         0 0 0 0.000005 0;
         0 0 0 0 0.000005];
end
[RE, Re] = eig(R);

% Measurement type and noise
if ~exist('meas','var')
    meas = 2; % 1 - range, 2 - bearing, 3 - both
end

if ~exist('Q','var')
    switch(meas)
        case 1
            Q = 0.01;
        case 2
            Q = 0.01;
        case 3
             Q = [0.01 0; 
                  0 0.01];
    end
end
[QE, Qe] = eig(Q);

% Number of particles
if ~exist('D','var')
    D = 550;
end
% Prior - uniform over [-1 1] position and [-pi/6 pi/6] heading
if ~exist('mu','var')
    mu = [0 4 0 -5 2];
end


X = diag(mu)*(2*rand(5,D)-1);
X0 = X;
Xp = X;
w = zeros(1,D);
[pRE, pRe] = eig(10*R);

% Feature Map
map = [ -10 0;  
         10 0;
         10 20;
         -10 20];

% Simulation Initializations
n = length(x0);
x = zeros(n+2,length(T));
x(:,1) = [x0(1), x0(2), x0(3), vx, vy];
m = length(Q(:,1));
y = zeros(m,length(T));
mf = zeros(2,length(T));

figure(1);clf; hold on;
plot(map(:,1),map(:,2),'go', 'MarkerSize',10,'LineWidth',2);
plot(x(1,1),x(2,1), 'ro--')
for dd=1:D
    plot(X(1,dd),X(2,dd),'b.')
end
axis equal
axis([-15 25 -15 25]);
title('Particle Filter Localization')
F(1) = getframe;


%% Main loop
for t=2:length(T)
    %% Simulation
    % Generate a motion disturbance
    e = RE*sqrt(Re)*randn(5,1);
    % Update state
    x(:,t) = [x(1,t-1)+(u(1,t)*cos(x(3,t-1))+ x(4, t-1))*dt;
              x(2,t-1)+(u(1,t)*sin(x(3,t-1))+ x(5, t-1))*dt;
              x(3,t-1)+u(2,t)*dt;
              x(4, t-1);
              x(5, t-1)] + e;
    
    % Take measurement
    % Pick feature
    mf(:,t) = closestfeature(map,x(:,t));
    % Generate a measurement disturbance
    d = QE*sqrt(Qe)*randn(m,1);
    % Determine measurement
    switch(meas) 
        case 1
            y(:,t) = (sqrt((mf(1,t)-x(1,t))^2 + (mf(2,t)-x(2,t))^2));% + d;
        case 2 
            y(:,t) = (atan2(mf(2,t)-x(2,t),mf(1,t)-x(1,t))-x(3,t)) + d;
        case 3
            y(:,t) = [sqrt((mf(1,t)-x(1,t))^2 + (mf(2,t)-x(2,t))^2);
                      atan2(mf(2,t)-x(2,t),mf(1,t)-x(1,t))-x(3,t)];% + d;
    end
    %% Particle filter estimation
    for dd=1:D
        e = pRE*sqrt(pRe)*randn(5,1);
        Xp(:,dd) = [X(1,dd)+(u(1,t)*cos(X(3,dd))+ X(4, dd))*dt;
                    X(2, dd)+(u(1,t)*sin(X(3,dd))+ X(5, dd))*dt;
                    X(3,dd)+u(2,t)*dt;
                    X(4, dd);
                    X(5, dd)] + e;

        

        d = QE*sqrt(Qe)*randn(m,1);
        switch(meas)
            case 1
                hXp = [sqrt((mf(1,t)-Xp(1,dd))^2 + (mf(2,t)-Xp(2,dd))^2)];% + d;
            case 2
                hXp = [atan2(mf(2,t)-Xp(2,dd),mf(1,t)-Xp(1,dd))-Xp(3,dd)] + d;
            case 3
                hXp = [sqrt((mf(1,t)-Xp(1,dd))^2 + (mf(2,t)-Xp(2,dd))^2);
                         atan2(mf(2,t)-Xp(2,dd),mf(1,t)-Xp(1,dd))-Xp(3,dd)];% + d;
        end
        w(dd) = mvnpdf(y(:,t),hXp,Q);
    end
    W = cumsum(w);
    for dd=1:D
        seed = max(W)*rand(1);
        X(:,dd) = Xp(:,find(W>=seed,1));
    end
    muParticle = mean(X');
    SParticle = cov(X');

    %muParticle_S(:,dd) = muParticle;
    
     %% Plot results
     figure(1);clf; hold on;
     plot(map(:,1),map(:,2),'go', 'MarkerSize',10,'LineWidth',2);
     plot(mf(1,t),mf(2,t),'mx', 'MarkerSize',10,'LineWidth',2)
     plot(x(1,1:t),x(2,1:t), 'ro--')
     if (meas==1) circle(x(1:2,t), y(1,t)); end
     if (meas==2) plot([x(1,t) x(1,t)+10*cos(y(1,t)+x(3,t))], [ x(2,t) x(2,t)+10*sin(y(1,t)+x(3,t))], 'c');end
     if (meas==3) plot([x(1,t) x(1,t)+y(1,t)*cos(y(2,t)+x(3,t))], [ x(2,t) x(2,t)+y(1,t)*sin(y(2,t)+x(3,t))], 'c');end
     try
        error_ellipse(SParticle(1:2,1:2),muParticle(1:2),.999);
     catch
        disp('error_ellipse failed')
        SParticle(1:2,1:2)
        muParticle(1:2)
     end
     for dd=1:D
         plot(X(1,dd),X(2,dd),'b.')
     end
     axis equal
     axis([-15 25 -15 25]);
     title('Particle Filter Localization')
     F(t) = getframe;
     writeVideo(v, F(t));
end
close(v);