% %======================== Explanation of code ===========================% %
% % This is a slightly more advanced version of the typical PNG homing
% algorithm used for target tracking.
% Target is stationary in this case.
% This is for anyone who wants to understand the basics of how homing
% algorithms are used in the aeronautical industry, with software packages
% such as MATLAB. 

% % Get launch angle (theta) from user, then get initial x and y coordinates
% % Constants are VELOCITY (v), N, DESIGNATED TIME (Td) 
% % Variables are x and y 
% % LOS, a, L are always changing, so need to INCLUDE THEM IN A LOOP
% % Output will be x and y, ALSO ALWAYS CHANGING
% % Just Click 'Run'

% %============================== MAIN CODE ===================================%
global  Vtarget Vuav LOS_0 LOS_1 LOS_2 theta0 td Rrel_0 Rrel_1 Rrel_2 psi0 psi1 psi2 k0 k1 k2
 

% %=========== For Target coordinates AT [0,0] ===========% %
% %=========== uncomment to plot ===========% %
% %=========== controlled constants ===========% %
Vuav = 400;
Vtarget = 200;
theta = 10; 
theta0 = theta/180*pi;
targetx0 = 0; 
targety0 = 0; 
td = 50;
k0 = 60;
k1 = 60;
k2 = 50;

% % Starting Coordintaes
psi0 = -120; 
psi0 = psi0/180*pi;
xuav0 = 7000; 
yuav0 = 4000; 

psi1 = -140; 
psi1 = psi1/180*pi;
xuav1 = 8000; 
yuav1 = 1000; 

psi2 = 160; 
psi2 = psi2/180*pi;
xuav2 = 6000; 
yuav2 = -3500; 


% % =========== IMPORTANT CONDITION: TIME TAKEN MUST BE LESS THAN DESIGNATED TIME ===========% % 
LOS_0 = atan2((targety0-yuav0),(targetx0-xuav0));
Rrel_0 = sqrt((targetx0-xuav0)^2 + (targety0-yuav0)^2);

LOS_1 = atan2((targety0-yuav1),(targetx0-xuav1));
Rrel_1 = sqrt((targetx0-xuav1)^2 + (targety0-yuav1)^2);
%  
LOS_2 = atan2((targety0-yuav2),(targetx0-xuav2));
Rrel_2 = sqrt((targetx0-xuav2)^2 + (targety0-yuav2)^2);


% % =========== Initial Conditions are for nondim variables ===========% %
tspan =[0: 0.0001: 1]; 
IC0 = [targetx0/Rrel_0, targety0/Rrel_0, xuav0/Rrel_0, yuav0/Rrel_0, 1, LOS_0, psi0]; 
IC1 = [targetx0/Rrel_1, targety0/Rrel_1, xuav1/Rrel_1, yuav1/Rrel_1, 1 ,LOS_1, psi1]; 
IC2 = [targetx0/Rrel_2, targety0/Rrel_2, xuav2/Rrel_2, yuav2/Rrel_2, 1, LOS_2, psi2]; 


% %=========== Function Call ===========% %
[t,ARR0] =ode45(@odefcn_movingtarget, tspan, IC0);
[t,ARR1] =ode45(@odefcn_movingtarget1, tspan, IC1);
[t,ARR2] =ode45(@odefcn_movingtarget2, tspan, IC2);

figure
hold on
% %=========== Plot X vs Y displacement ===========% %
plot(ARR0(:,1)*Rrel_0, ARR0(:,2)*Rrel_0, 'x' , 'Color', 'k')
plot(ARR0(:,3)*Rrel_0, ARR0(:,4)*Rrel_0,'LineWidth', 2 , 'Color', 'k')
plot(ARR1(:,3)*Rrel_1, ARR1(:,4)*Rrel_1,'LineWidth', 2 , 'Color', 'k')
plot(ARR2(:,3)*Rrel_2, ARR2(:,4)*Rrel_2,'LineWidth', 2 , 'Color', 'k')
xlabel('x displacement (m)');
ylabel('y displacement (m)');
title('Trajectory of UAVs on a Moving Target X ');

% %=========== Plot LOS Angle vs Time ===========% %
% %=========== Uncomment to selectively plot ===========% %
% time0 = linspace(0, td, length(ARR0(:, 6)));
% time1 = linspace(0, td, length(ARR1(:, 6)));
% time2 = linspace(0, td, length(ARR2(:, 6)));
% plot(time0, ARR0(:,6), 'LineWidth', 2 , 'Color', 'k')
% plot(time1, ARR1(:,6), 'LineWidth', 2 , 'Color', 'k')
% plot(time2, ARR2(:,6), 'LineWidth', 2 , 'Color', 'k')
% xlabel('Time');
% ylabel('Line of Sight Angle (deg)');
% title('Line-Of-Sight Angle Vs Time');

% %=========== Plot Relative Distance vs Time ===========% %
% %=========== Uncomment to selectively plot ===========% %
% plot(time0, ARR0(:,5)*Rrel_0, 'LineWidth', 2 , 'Color', 'k')
% plot(time1, ARR1(:,5)*Rrel_1, 'LineWidth', 2 , 'Color', 'k')
% plot(time2, ARR2(:,5)*Rrel_2, 'LineWidth', 2 , 'Color', 'k')
% xlabel('Time');
% ylabel('Relative Distance');
% title('Relative Distance Vs Time');

% %============================== End of Main Code ===================================% %


% %============================== FUNCTION CODE ===================================% %
% %========= IMPORTANT TERMS ==========% %
% Td: Designated impact time
% Tgo_des: Designated time-to-go
% Tgo: Approximated time-to-go

% %============================== FUNCTION 1 ===================================% %
function dARRdt =  odefcn_movingtarget(t, ARR)
global  Vtarget Vuav LOS_0 LOS_1 LOS_2 theta0 td Rrel_0 Rrel_1 Rrel_2 psi0 psi1 psi2 k0 k1 k2

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
dARRdt(5) = (vtarget_nd*cos(theta0-ARR(6)) - vuav_nd*cos(ARR(7)-ARR(6))); %relative distance
dARRdt(6) = ((vtarget_nd*sin(theta0-ARR(6)) - vuav_nd*sin(ARR(7)-ARR(6)))/ARR(5));% los angle UAV
angle_diff = ARR(7) - ARR(6); % flight path angle - los angle

tgo = (td - t)/td; %actual time to go
tgo_cap_1 = ((ARR(5)/(vuav_nd + vtarget_nd))*(1 + (angle_diff^2)/(4*N-2)));    %estimated time to go when diff is more than 90 deg
tgo_cap_2 = ((ARR(5)/(vuav_nd - vtarget_nd))*(1 + (angle_diff^2)/(4*N-2)));     %estimated time to go when diff is less than 90 deg
initial_angle_diff = psi0 - LOS_0;

    if  abs(initial_angle_diff) > pi/2
        tgo_cap_org = (1/(vuav_nd + vtarget_nd))*(1 + (initial_angle_diff^2)/(4*N-2));
    else
        tgo_cap_org = (1/(vuav_nd - vtarget_nd))*(1 + (initial_angle_diff^2)/(4*N-2));
    end

png = (1+N/cos(angle_diff))*vuav_nd*dARRdt(6);

    if abs(angle_diff) > pi/2
        tgo_cap = tgo_cap_1;
        a = png + (k0/(1*tgo_cap_org))*ARR(5)*(tgo-tgo_cap);
    else if abs(angle_diff) <= pi/2
        tgo_cap = tgo_cap_2;
        a = png + (k0/(1*tgo_cap_org))*ARR(5)*(tgo-tgo_cap);
        
       
        dARRdt(7) = (a/vuav_nd); %psi
       
        end        
    end
end


% %============================== FUNCTION 2 ===================================% %
function dARRdt =  odefcn_movingtarget1(t, ARR)
global  Vtarget Vuav LOS_0 LOS_1 LOS_2 theta0 td Rrel_0 Rrel_1 Rrel_2 k psi0 psi1 psi2 k0 k1 k2

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
dARRdt(5) = (vtarget_nd*cos(theta0-ARR(6)) - vuav_nd*cos(ARR(7)-ARR(6))); %relative distance
dARRdt(6) = ((vtarget_nd*sin(theta0-ARR(6)) - vuav_nd*sin(ARR(7)-ARR(6)))/ARR(5));% los angle UAV
angle_diff = ARR(7) - ARR(6); % flgiht path angle - los angle

tgo = (td - t)/td; %actual time to go
tgo_cap_1 = ((ARR(5)/(vuav_nd + vtarget_nd))*(1 + (angle_diff^2)/(4*N-2)));    %estimated time to go when diff is more than 90 deg
tgo_cap_2 = ((ARR(5)/(vuav_nd - vtarget_nd))*(1 + (angle_diff^2)/(4*N-2)));     %estimated time to go when diff is less than 90 deg
initial_angle_diff = psi1 - LOS_1;

    if  abs(initial_angle_diff) > pi/2
        tgo_cap_org = (1/(vuav_nd + vtarget_nd))*(1 + (initial_angle_diff^2)/(4*N-2));
    else
        tgo_cap_org = (1/(vuav_nd - vtarget_nd))*(1 + (initial_angle_diff^2)/(4*N-2));
    end

png = (1+N/cos(angle_diff))*vuav_nd*dARRdt(6);

   if abs(angle_diff) > pi/2
        tgo_cap = tgo_cap_1;
        a = png + (k1/(1*tgo_cap_org))*ARR(5)*(tgo-tgo_cap);
    else if abs(angle_diff) <= pi/2
        tgo_cap = tgo_cap_2;
        a = png + (k1/(1*tgo_cap_org))*ARR(5)*(tgo-tgo_cap);

        dARRdt(7) = (a/vuav_nd); %psi
        end
   end
end


% %============================== FUNCTION 3 ===================================% %
function dARRdt =  odefcn_movingtarget2(t, ARR)
global  Vtarget Vuav LOS_0 LOS_1 LOS_2 theta0 td Rrel_0 Rrel_1 Rrel_2 k psi0 psi1 psi2 k0 k1 k2

dARRdt = zeros(7, 1); 
N = 3;

% % Non-dimensionalise Variables
vuav_nd = Vuav*td/Rrel_2;
vtarget_nd = Vtarget*td/Rrel_2;

% % ODE Solver
dARRdt(1) = vtarget_nd *cos(theta0); %xtarget
dARRdt(2) = vtarget_nd *sin(theta0); %ytarget
dARRdt(3) = vuav_nd*cos(ARR(7)); %xuav
dARRdt(4) = vuav_nd*sin(ARR(7)); %yuav
dARRdt(5) = (vtarget_nd*cos(theta0-ARR(6)) - vuav_nd*cos(ARR(7)-ARR(6))); %relative distance
dARRdt(6) = ((vtarget_nd*sin(theta0-ARR(6)) - vuav_nd*sin(ARR(7)-ARR(6)))/ARR(5));% los angle UAV
angle_diff = ARR(7) - ARR(6); % flgiht path angle - los angle

tgo = (td - t)/td; %actual time to go
tgo_cap_1 = ((ARR(5)/(vuav_nd + vtarget_nd))*(1 + (angle_diff^2)/(4*N-2)));    %estimated time to go when diff is more than 90 deg
tgo_cap_2 = ((ARR(5)/(vuav_nd - vtarget_nd))*(1 + (angle_diff^2)/(4*N-2)));     %estimated time to go when diff is less than 90 deg
initial_angle_diff = psi2 - LOS_2;

    if  abs(initial_angle_diff) > pi/2
        tgo_cap_org = (1/(vuav_nd + vtarget_nd))*(1 + (initial_angle_diff^2)/(4*N-2));
    else
        tgo_cap_org = (1/(vuav_nd - vtarget_nd))*(1 + (initial_angle_diff^2)/(4*N-2));
    end

png = (1+N/cos(angle_diff))*vuav_nd*dARRdt(6);

    if abs(angle_diff) > pi/2
        tgo_cap = tgo_cap_1;
        a = png + (k2/(1*tgo_cap_org))*ARR(5)*(tgo-tgo_cap);
    else if abs(angle_diff) <= pi/2
        tgo_cap = tgo_cap_2;
        a = png + (k2/(1*tgo_cap_org))*ARR(5)*(tgo-tgo_cap);
   
        dARRdt(7) = (a/vuav_nd); %psi
        end
    end 
end

% %============================== End of Function Code ===================================% %