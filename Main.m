%Software Created by Dimitris Alexandridis to be used in collaborative robotics project
%Contributors: Andrew Lam, Edward Saunders, Dimitris Alexandridis


%WARNING: Before this script is run, the wheels must be lifted to the highest point.
%Setup

tic;

%Variable Declaration

x_motion_threshold = 35;
y_motion_threshold = 150;
x_speed = 15; %speed of horizontal interval motion
y_speed = 50; %speed of vertical motion

initial_pos = 13; %translates to 9.5cm distance from wall
moved = false; %bool: the coefficient of static friction has been exceeded
results = [];
current_pos = 5; %ultrasonic reading 0.7s after lifting wheels
prev_pos = 0; %ultrasonic reading before lifting wheels
pos_difference = 1; %difference of prev_pos and current_pos to verify slip

% Establish connection with the NXT brick
MyNXT = COM_OpenNXT();
COM_SetDefaultNXT(MyNXT);
disp('Connection to NXT brick successful');

% Set up params for sychronous motion of motors A,B (front/back gearing)
mA = NXTMotor('A');
mB = NXTMotor('B');
mA.SmoothStart = 1;
mA.SpeedRegulation = 1;
mA.Power = -x_speed;
mB.SmoothStart = 1;
mB.SpeedRegulation = 1;
mB.Power = x_speed;
disp('Params for wheel motion successfully set.');

% Set up parameters for motor C (lift)
mC = NXTMotor('C'); % associate mA with the motor connected to port A
mC.SmoothStart = 1; % do not use the smooth start option
mC.SpeedRegulation = 0; % do not use the speed regulation option
mC.Power = y_speed; % drive the motor at 20% power
mC.Stop('brake');
disp('Params for lift motion successfully set.');

% Establish connection to ultrasonic range sensor connected to sensor port 1

OpenUltrasonic(SENSOR_1);
disp('Established connection to ultrasonic range sensor');

% Take Robot to initial position (make sure string is streched to its normal equilibrium pos)

%1. Lower wheels
disp('Lowering Wheels.');
mC.Power = -y_speed;
mC.SendToNXT();
while (mC.ReadFromNXT().Position > -y_motion_threshold)
end
mC.Stop('brake');
disp('Wheels lowered.'); 

%2. Move to initial position
mA.SendToNXT();
mB.SendToNXT();
disp('Moving to initial position.');
while (GetUltrasonic(SENSOR_1) < initial_pos)
end
mA.Stop('brake');
mB.Stop('brake');
disp('Moved to initial position.');

%3. Lift Wheels
disp('Lifting Wheels.');
mC.Power = y_speed;
mC.SendToNXT();
temp_time = toc;
while (mC.ReadFromNXT().Position < y_motion_threshold)

    if (toc - temp_time > 0.5)
        break;
    end
    
end

mC.Stop('brake');
disp('Wheels lifted.');
disp('Success. Initial position procedure complete.') 

%Main_Loop
disp('**Entering Main Loop**');
tic;
disp('Resetting clock.');
moved = false;
mA.ResetPosition();
mB.ResetPosition();
mC.ResetPosition();

disp('**Initiating horizontal motion in intervals.**');
disp('');

while (moved == false)
    disp('Lowering Wheels.');
    mC.Power = -y_speed;
    mC.SendToNXT();
    while (mC.ReadFromNXT().Position > -y_motion_threshold)
    end
    mC.Stop('brake');
    disp('Wheels lowered.'); 
    
    X = ['Previous position: ', num2str(prev_pos)];
    disp(X);
    Y = ['Current position: ', num2str(current_pos)];
    disp(Y);
    
    if ((prev_pos - current_pos) > pos_difference)
        moved = true;
        results = [results; toc  GetUltrasonic(SENSOR_1) moved];
        disp('Initiating calculation procedure')
        break;
    else
        moved = false;
        results = [results; toc  GetUltrasonic(SENSOR_1) moved];
        disp('Robot not slipped.')
        disp('------------------');
    end
    
    %Move to next test position
    mA.SendToNXT();
    mB.SendToNXT();
    while (mB.ReadFromNXT().Position <= x_motion_threshold)
    end 
    mA.Stop('brake')
    mA.ResetPosition();
    mB.Stop('brake')
    mB.ResetPosition();
    prev_pos = GetUltrasonic(SENSOR_1);
    
    %Lift wheels
    disp('Lifting Wheels.');
    mC.Power = y_speed;
    mC.SendToNXT();
    temp_time = toc;
    while (mC.ReadFromNXT().Position < y_motion_threshold)

        if (toc - temp_time > 0.5)
            break;
        end
    
    end
    
    pause(0.7);
    current_pos = GetUltrasonic(SENSOR_1);
    
end

%Reset and lift wheels
pause(0.5);
disp('Lifting Wheels.');
    mC.Power = y_speed;
    mC.SendToNXT();
    temp_time = toc;
    while (mC.ReadFromNXT().Position < y_motion_threshold)

        if (toc - temp_time > 0.5)
            break;
        end
    end

temp_var = ['Robot slipped between ', num2str(prev_pos), ' and ', num2str(current_pos)];

disp(temp_var)

temp_var = ['Extension is between ', num2str(prev_pos-9.5), ' and ', num2str(current_pos-9.5)];
disp(temp_var)

av_extension = (prev_pos + current_pos - 15) / 2;
coefficient = av_extension * 0.01 * 25 / (0.882 * 9.81);

temp = ['Coefficient of static friction is: ', num2str(coefficient)];
disp(temp);

disp('TERMINATING')


%Plot data distance vs time
extensions = results(:,1) - 9.5;
plot(extensions(:,1), results(:,2));
grid on;
xlabel('time (s)');
ylabel('Extension of spring');

%Save data to file
save(datestr(datetime('now')))

% Close connection to the ultrasonic sensor
CloseSensor(SENSOR_1);

% Close connection to the NXT brick
COM_CloseNXT(MyNXT);
