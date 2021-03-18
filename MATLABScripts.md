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

## Optimizer

## Analyser

## Plotter

| [home](index) | [back](walkingRobot) |
