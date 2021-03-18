---
title: MATLAB Scripts
filename: MATLABScripts
description: Walking Robot
---

| [home](index) | [back](walkingRobot) |

---

# Scripts

## Contact Force
```Matlab
function [N, f] = Contact_force(vx, vz, pz)

k = 1e4;

c = 1e2;

mu = 0.5;

    if pz < 0
        N = -k*pz - c*vz;
    else
        N = 0;
    end

    
    if vx > 0.01
        if pz > -0.001
            if pz < 0.001
    
                f = -mu*N;
                
            else
                f = 0;
                
            end
        else
            
            f = 0;
            
        end
    else
        
        f = 0;
        
    end
           
    
    if f > 6
        f = 6;
        
    elseif f < -6
        
        f = -6;
  
    end
end
```
## Robot Parameters
```Matlab
% Robot Parameters

%% General Parameters
body_density = 0.514;
leg_density = 2.471;
foot_density = 0.7;
world_damping = 0.25;
world_rot_damping = 0.25;
if ~exist('actuatorType', 'var')
    actuatorType = 1;
end

%% Inputs
gait_period = 1.5;
time = linspace(0,gait_period,6);
ankle_motion    = deg2rad([40.08    40.08   24.155   8.23       12      24.155]);
knee_motion     = deg2rad([90       48.18   48.21    48.24      90      90]);
hip_motion      = deg2rad([20       8.2     24.1     40         60      40]);

ankle_motion1    = deg2rad([8.23       12      24.155   40.08    40.08   24.155]);
knee_motion1     = deg2rad([48.24      90      90       90       48.18   48.21]);
hip_motion1      = deg2rad([40         60      40       20       8.2     24.1]);

%% Actuator Constants
motion_time_constant = 0.01;

%% plane
plane_x = 1000;
plane_y = 75;
plane_z = 2.5;

%% Leg Parameters
leg_radius = 0.75;
lower_leg_length = 10;
upper_leg_length = 10;

%% Foot Parameters
foot_x = 8;
foot_y = 6;
foot_z = 1;

foot_offset = [-1 0 0];

%% Body Parameters
body_x = 10;
body_y = 5;
body_z = 15;

body_pos_x = 5;
body_pos_y = 0.5;
body_pos_z = -4;

height = (body_y/2) - body_pos_z + upper_leg_length + (plane_z/2) + lower_leg_length + 2*(foot_z);


```

## Optimizer
```Matlab
loop_number = 0;
tic

for n = [-20:20] 
    for m = [-20:20]
        for p = [-20:20]
            
            loop_number = loop_number + 1

            ankle_motion    = deg2rad([40.08+n    40.08+n   24.155+n   8.23+n       12+n      24.155+n]');
            knee_motion     = deg2rad([90+m       48.18+m   48.21+m    48.24+m      90+m      90+m]');
            hip_motion      = deg2rad([20+p       8.2+p     24.1+p     40+p         60+p      40+p]');
            
            ankle_motion1    = deg2rad([8.23+n       12+n      24.155+n   40.08+n    40.08+n   24.155+n]);
            knee_motion1     = deg2rad([48.24+m      90+m      90+m       90+m       48.18+m   48.21+m]);
            hip_motion1      = deg2rad([40+p         60+p      40+p       20+p       8.2+p     24.1+p]);


            s = sim('simple_robot');
            try
                torque_hip      = s.simout.signals.values(510:end);
                torque_knee     = s.simout1.signals.values(510:end);
                torque_ankle    = s.simout2.signals.values(510:end);
            catch 
                torque_hip      = 99;
                torque_knee     = 99;
                torque_ankle    = 99;
                
            end
            torque = abs(torque_hip) + abs(torque_knee) + abs(torque_ankle);
            
            max_hip(loop_number,1)     = max(abs(torque_hip));
            max_knee(loop_number,1)    = max(abs(torque_knee));
            max_ankle(loop_number,1)   = max(abs(torque_ankle));
            
            max_torque(loop_number,1) = max([max_hip(loop_number,1), max_knee(loop_number,1), max_ankle(loop_number,1)]);
          
            max_sum(loop_number,1) = max(torque);
           
        end
    end
end

toc
```
## Analyser
```Matlab
low_t = min(max_hip);

low_t_pos = find(max_hip == low_t);

ival = 1;

for i = 1:21
    if (ival+441) < low_t_pos
        
        ival = i*441;
        
    else
        
        ival = ival;
        
    end
end

jval =1;

for j = 1:21
    if ivval + (jval+21) < low_t_pos
        
        jval = j*21;
        
    else
        
        jval = jval;
        
    end
end

kval = low_t_pos - ival - jval;

i = ival/441;
j = jval/21;
p = kval;

 i = i-11;
 j = j-11;
 p = p-11;

ankle_motion    = deg2rad([40.08+i    40.08+i   24.155+i   8.23+i       12+i      24.155+i]');
knee_motion     = deg2rad([90+j       48.18+j   48.21+j    48.24+j      90+j      90+j]');
hip_motion      = deg2rad([20+p       8.2+p     24.1+p     40+p         60+p      40+p]');

ankle_motion1    = deg2rad([8.23+i       12+i      24.155+i     40.08+i    40.08+i   24.155+i]');
knee_motion1     = deg2rad([48.24+j      90+j      90+j     90+j       48.18+j   48.21+j]');
hip_motion1      = deg2rad([40+p         60+p      40+p     20+p       8.2+p     24.1+p]');

clear('i','j','k');
```
## Plotter
```Matlab
L_k = round(length(s.simout2.signals.values) - (length(s.simout2.signals.values)/5)*2);
S_k = (length(s.simout2.signals.values) - L_k) + 1;
S_k = s.simout2.signals.values(S_k:end);
S_k = transpose(S_k);

L_a = round(length(s.simout1.signals.values) - (length(s.simout1.signals.values)/5)*2);
S_a = (length(s.simout1.signals.values) - L_a) + 1;
S_a = s.simout1.signals.values(S_a:end);
S_a = transpose(S_a);

L_h = round(length(s.simout.signals.values) - (length(s.simout.signals.values)/5)*2);
S_h = (length(s.simout.signals.values) - L_h) + 1;
S_h = s.simout.signals.values(S_h:end);
S_h = transpose(S_h);

t = linspace(2,5,L_k);

figure

hold on
grid on
 ylim([-7,7]);
plot(t,S_k)
plot(t,S_a)
plot(t,S_h)
 xlabel('Time (s)')
 ylabel('Torque (Nm)')
 legend('Knee', 'Ankle', 'Hip','location','southeastoutside')

```
| [home](index) | [back](walkingRobot) |
