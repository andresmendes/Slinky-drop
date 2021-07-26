%% Slinky drop - Lumped-element model
% Simulation and animation of a slinky drop using a lumped-element model.
%
%%

clear ; close all ; clc

%% Parameters

% Slinky
N = 10;                                 % Number of elements
k = 10;                                 % Spring constant               [N/m]
m = 0.01;                               % Mass                          [kg]
g = 9.81;                               % Gravity                       [m/s2]

% Video
playback_speed = 0.005;                   % Speed of playback
tF      = 5;                            % Final time                    [s]
fR      = 30/playback_speed;            % Frame rate                    [fps]
dt      = 1/fR;                         % Time resolution               [s]
time    = linspace(0,tF,tF*fR);         % Time                          [s]

%% First phase
c = 0;                                  % Number of collisions

% Creating state space model
[A,B] = create_model(k,m,N,c,g);

% Initial condition. Static initial condition
A_static = A(N+1+1:end,2:N);
B_static = B(N+1+1:end);
X = linsolve(A_static,-B_static);

Z0 = [0 ; X ; zeros(N,1)];

X_CG = mean([0 ; X]); % CG initial position

% Simulation
options = odeset('Events',@(t,z) mass_collision(t,z,c));
[TOUT,ZOUT] = ode45(@(t,z) model_springs(t,z,A,B),time,Z0,options);

% Initialization
TOUT_total = TOUT;
ZOUT_total = ZOUT;

%% Subsequent phases

for c = 1:N-2
    % c is the number of collisions, for instance, when element 1 colide
    % with element 2.

    % update model according to the number of collisions.
    [A,B] = create_model(k,m,N,c,g);

    % Conservation of momentum:
    v = (c*ZOUT(end,N+c) + ZOUT(end,N+c+1))/(c+1);

    % The initial condition of the next phase is equal to the final
    % condition of the last phase.
    Z0 = ZOUT(end,:)';
    % However, the speed of all elements together (after collision) is the
    % same:
    Z0(N+1:N+1+c) = v; 

    % Simulation
    options = odeset('Events',@(t,z) mass_collision(t,z,c));
    [TOUT,ZOUT] = ode45(@(t,z) model_springs(t,z,A,B),time,Z0,options);

    % Concatenate
    TOUT_total = [TOUT_total ; TOUT_total(end)+TOUT];
    ZOUT_total = [ZOUT_total ; ZOUT];
end

% Extend simulation time in 10%
time_add = linspace(0,0.1*TOUT_total(end),0.1*TOUT_total(end)*fR)';

%% Fourth phase
% Collapsed slinky falling.

c = N-1;
v = (c*ZOUT(end,N+c) + ZOUT(end,N+c+1))/(c+1);

% Using first element for position and last element for speed
ZOUT_pos = ZOUT(end,1) + v.*time_add + g/2*time_add.^2;
ZOUT_vel = v + g*time_add;
% For all elements
ZOUT_pos_all = repmat(ZOUT_pos,1,N);
ZOUT_vel_all = repmat(ZOUT_vel,1,N);

ZOUT_final_phase = [ZOUT_pos_all ZOUT_vel_all];

% Concatenate
TOUT_total = [TOUT_total ; TOUT_total(end) + time_add];
ZOUT_total = [ZOUT_total ; ZOUT_final_phase];

%% Center of gravity

% Uniformly Accelerated Motion (UAM)
x_cg = X_CG + g/2*TOUT_total.^2;
v_cg = g*TOUT_total;

%% Results

% Speed of CG
% v = m * sum(YOUT_total(:,N+1:2*N),2)/(N*m);

figure
hold on ; grid on ; box on
set(gca,'Ydir','reverse')
plot(TOUT_total,ZOUT_total(:,1:N))
plot(TOUT_total,x_cg,'c')
xlabel('tempo [s]')
ylabel('position [m]')

figure
hold on ; grid on ; box on
% set(gca,'Ydir','reverse')
plot(TOUT_total,ZOUT_total(:,N+1:end))
% plot(TOUT_total,v,'--')
% plot(TOUT_total(end),v_final,'g*')
plot(TOUT_total,v_cg,'c')
xlabel('tempo [s]')
ylabel('speed [m/s]')

%% Animation

c = cool(N); % Colormap

figure
set(gcf,'Position',[50 50 1280 720]) % YouTube: 720p
% set(gcf,'Position',[50 50 854 480]) % YouTube: 480p
% set(gcf,'Position',[50 50 640 640]) % Social

% Create and open video writer object
v = VideoWriter('slinky_drop.mp4','MPEG-4');
v.Quality   = 100;
% v.FrameRate = fR;
open(v);

for i=1:length(TOUT_total)
    
    subplot(2,2,1)
        cla
        set(gca,'Ylim',[0 max(ZOUT_total(:,1))]) % Using first element
        hold on ; grid on ; box on
        set(gca,'Ydir','reverse')
        for j=1:N
            plot(TOUT_total,ZOUT_total(:,j),'color',c(j,:),'LineWidth',2)
        end
        plot(TOUT_total,x_cg,'k')
        plot([TOUT_total(i) TOUT_total(i)],[0 max(ZOUT_total(:,1))],'k--')
        xlabel('Time [s]')
        ylabel('Position [m]')

    subplot(2,2,3)
        cla
        set(gca,'Ylim',[0 max(ZOUT_total(:,N+1))]) % Using first element
        hold on ; grid on ; box on
        for j=1:N
            plot(TOUT_total,ZOUT_total(:,N+j),'color',c(j,:),'LineWidth',2)
        end
        plot(TOUT_total,v_cg,'k')
        plot([TOUT_total(i) TOUT_total(i)],[0 max(ZOUT_total(:,N+1))],'k--')
        xlabel('Time [s]')
        ylabel('Speed [m/s]')
    
    subplot(2,2,[2 4])
        cla
        hold on ; grid on ; box on
        set(gca,'Ydir','reverse')
        set(gca,'Ylim',[0 1.1*ZOUT_total(end,N)])
        set(gca,'Xtick',[])
    
        % Display in for loop: https://www.mathworks.com/matlabcentral/answers/323648-plot-with-displayname-in-for-loop
        for j=1:N-1
            % Spring
            ps = plot([0 0],[ZOUT_total(i,j) ZOUT_total(i,j+1)],'k:','Displayname','Spring');
        end
        for j=1:N
            pm(j) = plot(0,ZOUT_total(i,j),'o','color',c(j,:),'MarkerFaceColor',c(j,:),'MarkerSize',6,'Displayname',strcat('m',num2str(j)));
        end
        pc = plot(0,x_cg(i),'ko','MarkerFaceColor','k','MarkerSize',4,'Displayname','CG');
        
        % Source: https://www.mathworks.com/matlabcentral/answers/25381-how-to-show-partial-legend-in-figure
        legend([ps(end) pm pc])
        ylabel('Position [m]')
        title(strcat('Time=',num2str(time(i),'%.3f'),' s (Playback speed=',num2str(playback_speed),')'))
        
    frame = getframe(gcf);
    writeVideo(v,frame);

end

close(v);

function dz =  model_springs(~,z,A,B)
    dz = A*z + B;
end

function [A,B] = create_model(k,m,N,c,g)
    % Number of effective elements is N (actual). c is the number of
    % collisions. After collision, the elements together do not separate.
    % The model is always NxN, where N is the number of elements, even
    % after collisions.
    
    % https://www.mathworks.com/help/matlab/ref/diag.html
    A1 = -2*k/m*eye(N);             % Identity matrix
    A2 = k/m*diag(ones(N-1,1),+1);  % above diagonal
    A3 = k/m*diag(ones(N-1,1),-1);  % below diagonal

    A_model = A1 + A2 + A3;
    
    % Update first and last elements
    A_model(1,1) = -k/m;
    A_model(end,end) = -k/m;

    % Updating matrix after collision.
    if c >=1
        A_model(:,1:c) = 0; 
        A_model(1:1+c,1+c) = -k/((1+c)*m);  
        A_model(1:1+c,2+c) = k/((1+c)*m);   
    end

    B_model = g*ones(N,1);

    % https://www.mathworks.com/help/matlab/ref/blkdiag.html
    % Montando o modelo completo
    A = [zeros(N) eye(N) ; A_model zeros(N)];
    B = [zeros(N,1) ; B_model];

end

function [position,isterminal,direction] = mass_collision(~,y,c)
    position = y(1+c)-y(2+c);   % First 2 masses collide
    isterminal = 1;             % Halt integration 
    direction = 0;              % The zero can be approached from either direction
end