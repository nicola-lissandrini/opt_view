%% Target trajectory generation

clearvars

%% Pars

sample_time = 0.01;
end_time = 30;

A1 = 3;
A2 = A1*2;
f1 = 1/10;
f2 = 1/20;
v = 1;

%% Trajectory

% Still part

t1 = 0:sample_time:6;

traj{1}(:,1) = 0 * t1 - 8;
traj{1}(:,2) = 0 * t1;

% Linear part

t2 = 0:sample_time:0.1;

traj{2}(:,1) = v * t2;
traj{2}(:,2) = 0 * t2;

% First evasion

t3 = 0:sample_time:5;

traj{3}(:,1) = v * t3;
traj{3}(:,2) = A1 * (-sin (2*pi*f1*t3))

% Second larger evasion

t4 = 0:sample_time:10;

traj{4}(:,1) = v * t4;
traj{4}(:,2) = A2 * (sin (2*pi*f2*t4));

% Lateral movement

t5 = 0:sample_time:5;

traj{5}(:,1) = v*t5;
traj{5}(:,2) = (traj{4}(end,2)-traj{4}(end-1,2))/sample_time*t5;

% Concat

trajtot = traj{1};

for i=2:5
    trajtot = [trajtot; traj{i}+trajtot(end,:)];
end

%% smooth
if 0
[b, a] = butter (4, 0.01);
trajtot(:,2) = filter (b, a, trajtot(:,2));
end

 csvwrite ('/home/nicola/psc_ws/trajectory', trajtot)

%% Plot

plot (trajtot(:,1), trajtot(:,2));
grid on
grid minor
axis equal




















