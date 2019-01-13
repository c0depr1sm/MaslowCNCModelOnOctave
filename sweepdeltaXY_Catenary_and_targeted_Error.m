## Copyright (C) 2018 c0depr1sm
## 
## This program is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.

## Created: 2018-11-01

## What? This function compares the xy position achieved when comparing the ideal 
## model calculation with a similar one having a parameter error.
## it includes sag with the catenary formula, tension, flex, offsets, etc (see parameter lists)
## you can cheat several parameters (see cheat codes into inverse.m)

## How? It computes chain lengths on straign sprocket triangular kynematics, then refines 
## it with the catenary estimation and the parameter error of your choice. Coming up with a set of chain lengths.
## It then forward chain length back to XY coorinates (like the maslow firmware does), but this time with the no
## error model.
## It collects the difference between target and "acheived" XY coordinates. That is the estimated xy 
## errors on sled positioning.

## Usage: 
##  Just run the script. It yields 3 figures where the xy plane is a sweep of the workspace in mm. 
##  fig 1 is the xy distance from first estimation model to second estimation model. 
##  fig 2 is the x error added to the target position.
##  fig 3 is the y error added to the target position. 
##

## earth location parameters
accG = 9.8; ## m/s^2

## Maslow cnc parameters
workspaceHeight = 1222; ## mm
workspaceWidth = 1240; ## mm

#workSpace area to sweep
x_zone = [-1200:100:1200];
y_zone = [600:-100:-600];

#matrice de réponse a remplir
workspaceX = zeros(length(x_zone),length(y_zone));
workspaceY = zeros(length(x_zone),length(y_zone));
catenary_errorX = zeros(length(x_zone),length(y_zone));
catenary_errorY = zeros(length(x_zone),length(y_zone));
catenary_errorSize = zeros(length(x_zone),length(y_zone));

for i = [1:1:length(x_zone)]
  for j = [1:1:length(y_zone)]
    xposraw = x_zone(i);
    workspaceX(i,j) = xposraw;
    yposraw = y_zone(j);
    workspaceY(i,j) = yposraw;
    
    ## initial geometry and chains lenght estimation -> call first to get chain angles estimations
    [leftChainExtent_raw, rightChainExtent_raw] = inverse(xposraw, yposraw,"allcatenary_with_error");
    
    ## get back to x and y position but this time include second model (maybe some more details, or some machine simulated errors)
    [xpos,ypos] = ForwardKynematics (leftChainExtent_raw, rightChainExtent_raw, xposraw, yposraw, "allcatenary_with_error2"); 
    
    ## compute delta x and y and norm
    catenary_errorX(i,j) = -xposraw+xpos;
    catenary_errorY(i,j) = -yposraw+ypos;
    catenary_errorSize(i,j) = norm([catenary_errorX(i,j) catenary_errorY(i,j)]);
  endfor
endfor

## If scales are not right, you can edit scales here-under and copy the following lines into
## the octave command line

figure (1);
surf(workspaceX,workspaceY,catenary_errorSize);
haxe = gca();
set(haxe,"dataaspectratio", [200 200 0.5]);
title('xy distance when omiting kynematics details (mm)');
figure (2);
surf(workspaceX,workspaceY,catenary_errorX);
haxe = gca();
set(haxe,"dataaspectratio", [200 200 0.1]);
title('x error when omiting kynematics details (mm)');
figure (3);
surf(workspaceX,workspaceY,catenary_errorY);
haxe = gca();
set(haxe,"dataaspectratio", [200 200 0.1]);
title('y error when omiting kynematics details (mm)');
