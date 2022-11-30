% %======================== Explanation of code ===========================% %
% % This is a slightly more advanced version of the typical PNG homing
% algorithm used for target tracking.
% Target is moving with a constant speed in this case.
% This is for anyone who wants to understand the basics of how homing
% algorithms are used in the aeronautical industry, with software packages
% such as MATLAB. 


% % Get launch angle (theta) from user, then get initial x and y coordinates
% % Constants are VELOCITY (vtarget, vuav), N, DESIGNATED TIME (Td) 
% % Variables are x and y 
% % LOS, a, L are always changing, so you need to include them in an
% automated loop
% % Output will be x and y which are always changing over time


% %============================== MAIN CODE ===================================% % 
global  Vtarget Vuav LOS_0 LOS_1 LOS_2 theta0 td Rrel_0 Rrel_1 Rrel_2  k psi0 psi1 psi2 


% %=========== For Target coordinates NOT at [0,0] ===========% %
% %=========== uncomment to plot ===========% %
% %=========== controlled constants ===========% %
% Vuav = 300;
% Vtarget = 0;
% theta = 10; 
% theta0 = theta/180*pi;
% targetx0 = 7000; 
% targety0 = 7000; 
% td = 50;
% k = 50;
% 
% % % Starting Coordintaes
% psi0 = 140; 
% psi0 = psi0/180*pi;
% xuav0 = 0; 
% yuav0 = 0; 
% 
% psi1 = 80; 
% psi1 = psi1/180*pi;
% xuav1 = 0; 
% yuav1 = 0; 
% 
% psi2 = 110; 
% psi2 = psi2/180*pi;
% xuav2 = 0; 
% yuav2 = 0; 

% %=========== For Target coordinates AT [0,0] ===========% %
% %=========== uncomment to plot ===========% %
% %=========== controlled constants ===========% %
Vuav = 400;
Vtarget = 0;
theta = 0; 
theta0 = theta/180*pi;
targetx0 = 0; 
targety0 = 5000; 
td = 50;
k = 50;

% %=========== Starting Coordintaes ===========% %
psi0 = 140; 
psi0 = psi0/180*pi;
xuav0 = 0; 
yuav0 = -8000; 

psi1 = 120; 
psi1 = psi1/180*pi;
xuav1 = -6000; 
yuav1 = -7500; 

psi2 = 30; 
psi2 = psi2/180*pi;
xuav2 = -9000; 
yuav2 = 5000; 

% % =========== IMPORTANT CONDITION: TIME TAKEN MUST BE LESS THAN DESIGNATED TIME ===========% % 
LOS_0 = atan2((targety0-yuav0),(targetx0-xuav0));
Rrel_0 = sqrt((targetx0-xuav0)^2 + (targety0-yuav0)^2);

LOS_1 = atan2((targety0-yuav1),(targetx0-xuav1));
Rrel_1 = sqrt((targetx0-xuav1)^2 + (targety0-yuav1)^2);
%  
LOS_2 = atan2((targety0-yuav2),(targetx0-xuav2));
Rrel_2 = sqrt((targetx0-xuav2)^2 + (targety0-yuav2)^2);


% % =========== Initial Conditions are for nondim variables ===========% %
tspan =[0: 0.00001: 1]; 
IC0 = [targetx0/Rrel_0, targety0/Rrel_0, xuav0/Rrel_0, yuav0/Rrel_0, 1, LOS_0, psi0]; 
IC1 = [targetx0/Rrel_1, targety0/Rrel_1, xuav1/Rrel_1, yuav1/Rrel_1, 1 ,LOS_1, psi1]; 
IC2 = [targetx0/Rrel_2, targety0/Rrel_2, xuav2/Rrel_2, yuav2/Rrel_2, 1, LOS_2, psi2]; 


% %=========== Function Call ===========% %
[t,ARR0] =ode113(@odefcn_stntarget, tspan, IC0);
[t,ARR1] =ode113(@odefcn_stntarget1, tspan, IC1);
[t,ARR2] =ode113(@odefcn_stntarget2, tspan, IC2);


figure
hold on
% %=========== Plot X vs Y displacement ===========% %
plot(ARR0(:,1)*Rrel_0, ARR0(:,2)*Rrel_0 , 'Color', 'k')
plot(ARR0(:,3)*Rrel_0, ARR0(:,4)*Rrel_0, 'LineWidth', 2 , 'Color', 'k')
plot(ARR1(:,3)*Rrel_1, ARR1(:,4)*Rrel_1, 'LineWidth', 2 , 'Color', 'k')
plot(ARR2(:,3)*Rrel_2, ARR2(:,4)*Rrel_2, 'LineWidth', 2 , 'Color', 'k')
xlim([-inf targetx0])
xlabel('x displacement (m)');
ylabel('y displacement (m)');
title('Trajectory of UAVs on a Stationary Target X ');

% %=========== Plot LOS Angle vs Time ===========% %
% %=========== Uncomment to selectively plot ===========% %
% plot(t*td, ARR0(:,6))
% plot(t*td, ARR1(:,6))
% plot(t*td, ARR2(:,6))
% xlabel('Time');
% ylabel('Line of Sight Angle (deg)');
% title('Line-Of-Sight Angle Vs Time');

% %=========== Plot Relative Distance vs Time ===========% %
% %=========== Uncomment to selectively plot ===========% %
plot(t*td, ARR0(:,5))
plot(t*td, ARR1(:,5))
plot(t*td, ARR2(:,5))
xlabel('Time');
ylabel('Relative Distance');
title('Relative Distance Vs Time');
% %============================== End of Main Code ===================================% %



% %============================== FUNCTION CODE ===================================% %
% %========= IMPORTANT TERMS ==========% %
% Td: Designated impact time
% Tgo_des: Designated time-to-go
% Tgo: Approximated time-to-go

% %============================== FUNCTION 1 ===================================% %
function dARRdt =  odefcn_stntarget(t, ARR)
global  Vtarget Vuav LOS_0 LOS_1 LOS_2 theta0 td Rrel_0 Rrel_1 Rrel_2 k psi0 psi1 psi2 

dARRdt = zeros(7, 1); 
N = 3;

% % Non-dimensionalise Variables
vuav_nd = Vuav*td/Rrel_0;
vtarget_nd = Vtarget*td/Rrel_0;

% % ODE SOlver
dARRdt(1) = vtarget_nd *cos(theta0); %xtarget
dARRdt(2) = vtarget_nd *sin(theta0); %ytarget
dARRdt(3) = vuav_nd*cos(ARR(7)); %xuav
dARRdt(4) = vuav_nd*sin(ARR(7)); %yuav
dARRdt(5) = -vuav_nd*cos(ARR(6)-ARR(7)); %relative distance
dARRdt(6) = (vuav_nd*sin(ARR(6)-ARR(7)))/ARR(5);% los angle UAV
tgo = (td - t)/td; %actual time to go
initial_angle_diff = psi0 - LOS_0;
tgo_cap_org = (1/(vuav_nd))*(1 + (initial_angle_diff^2)/(4*N-2));
angle_diff = ARR(7) - ARR(6);     
a = N*vuav_nd*dARRdt(6);
tgo_cap = (ARR(5)/vuav_nd)*(1 +(angle_diff^2)/(4*N-2));
dARRdt(7) = (a/vuav_nd); %psi    
end


% %============================== FUNCTION 2 ===================================% %
function dARRdt =  odefcn_stntarget1(t, ARR)
global  Vtarget Vuav LOS_0 LOS_1 LOS_2 theta0 td Rrel_0 Rrel_1 Rrel_2 k psi0 psi1 psi2 

dARRdt = zeros(7, 1); 
N = 3;

% % Non-dimensionalise Variables
vuav_nd = Vuav*td/Rrel_1;
vtarget_nd = Vtarget*td/Rrel_1;

% % ODE SOlver   
dARRdt(1) = vtarget_nd *cos(theta0); %xtarget
dARRdt(2) = vtarget_nd *sin(theta0); %ytarget
dARRdt(3) = vuav_nd*cos(ARR(7)); %xuav
dARRdt(4) = vuav_nd*sin(ARR(7)); %yuav
dARRdt(5) = -vuav_nd*cos(ARR(6)-ARR(7)); %relative distance
dARRdt(6) = (vuav_nd*sin(ARR(6)-ARR(7)))/ARR(5);% los angle UAV
tgo = (td - t)/td; %actual time to go
initial_angle_diff = psi1 - LOS_1;
tgo_cap_org = (1/(vuav_nd))*(1 + (initial_angle_diff^2)/(4*N-2));
angle_diff = ARR(7) - ARR(6);
a = N*vuav_nd*dARRdt(6);
tgo_cap = (ARR(5)/vuav_nd)*(1 +(angle_diff^2)/(4*N-2));
dARRdt(7) = (a/vuav_nd); %psi
end


% %============================== FUNCTION 3 ===================================% %
function dARRdt =  odefcn_stntarget2(t, ARR)
global  Vtarget Vuav LOS_0 LOS_1 LOS_2 theta0 td Rrel_0 Rrel_1 Rrel_2 k psi0 psi1 psi2 

dARRdt = zeros(7, 1); 
N = 3;

% % Non-dimensionalise Variables
vuav_nd = Vuav*td/Rrel_2;
vtarget_nd = Vtarget*td/Rrel_2;

% % ODE SOlver
dARRdt(1) = vtarget_nd *cos(theta0); %xtarget
dARRdt(2) = vtarget_nd *sin(theta0); %ytarget
dARRdt(3) = vuav_nd*cos(ARR(7)); %xuav
dARRdt(4) = vuav_nd*sin(ARR(7)); %yuav
dARRdt(5) = -vuav_nd*cos(ARR(6)-ARR(7)); %relative distance
dARRdt(6) = (vuav_nd*sin(ARR(6)-ARR(7)))/ARR(5);% los angle UAV
tgo = (td - t)/td; %actual time to go
initial_angle_diff = psi2 - LOS_2;
tgo_cap_org = (1/(vuav_nd))*(1 + (initial_angle_diff^2)/(4*N-2));
angle_diff = ARR(7) - ARR(6);
a = N*vuav_nd*dARRdt(6);
tgo_cap = (ARR(5)/vuav_nd)*(1 +(angle_diff^2)/(4*N-2));
dARRdt(7) = (a/vuav_nd); %psi      
end


% %============================== End of Function Code ===================================% %