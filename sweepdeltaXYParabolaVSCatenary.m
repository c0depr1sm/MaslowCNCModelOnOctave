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

## What? This function compares the xy position achieved when comparing the parabola 
## approximation for chain sag and chain tension with the exact catenary formula.

## How? It computes chain lengths on straign sprocket tiragular kynematics, then refines 
## it with the parabola estimation like the current Maslow firmware. It then redoes it but 
## this time with an implementation of an successive approximation to fit the catenary function.
## It collects the difference between parabola and catenary xy errors likely occuring on sled positioning. 
## due to the more accurate catenary calculation.

## Limits? 
## it is important to adjust the sag correction factor (like the one in the maslow firmware).
## In the command window, get a plain left chain length value for the lower left corner by typing:
# inverse(1200, -600,"bare")
# ans =  3497.1
## you get the parabola version here (the one using the sag correction factor)
# inverse(1200, -600,"allparabola")
# ans =  3498.5
## and here you get the one using a catenary equation (relying on sled weigh and chain density)
# inverse(1200, -600,"allcatenary")
# ans =  3498.5
## Now you would want to get the parabola to match the catenary, so compute the chain length difference:
# inverse(1200, -600,"allparabola")(1)-inverse(1200, -600,"allcatenary")(1)
# ans =   -7.9500e-04
##
## if you don't get a very small difference on "parabola" and "catenary", tweak the inverse.m
## function parameter "chainSagCorrectionFactor".
##

## Usage: 
##  Just run the script. It yields 3 figures where the xy plane is a sweep of the workspace in mm. 
##  fig 1 is the xy distance from parabola estimation to catenary estimation. 
##  fig 2 is the x error to be added to the parabola estimation to reach the catenary. 
##  fig 3 is the y error to be added to the parabola estimation to reach the catenary.  
##
##  Then change the parameters in the list below and into inverse.m to set more specific conditions

## earth location parameters
accG = 9.8; ## m/s^2

## Maslow cnc parameters
workspaceHeight = 1222; ## mm
workspaceWidth = 1240; ## mm

#workSpace area to sweep
x_zone = [-1200:25:1200];
y_zone = [600:-25:-600];

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
    
    ## initial geometry and chains lenght estimation without sag -> call first to gete chain angles estimations
    [leftChainExtent_raw, rightChainExtent_raw] = inverse(xposraw, yposraw,"allparabola");
    
    ## get back x and y position but this time include "all" model details
    [xpos,ypos] = ForwardKynematics (leftChainExtent_raw, rightChainExtent_raw, xposraw, yposraw, "allcatenary"); 
    
    ## compute delta x and y and norm
    catenary_errorX(i,j) = xposraw-xpos;
    catenary_errorY(i,j) = yposraw-ypos;
    catenary_errorSize(i,j) = norm([catenary_errorX(i,j) catenary_errorY(i,j)]);
  endfor
endfor

figure (1);
surf(workspaceX,workspaceY,catenary_errorSize);
haxe = gca();
set(haxe,"dataaspectratio", [200 200 0.05]);
title('xy distance when omiting kynematics details (mm)');
figure (2);
surf(workspaceX,workspaceY,catenary_errorX);
haxe = gca();
set(haxe,"dataaspectratio", [200 200 0.05]);
title('x error when omiting kynematics details (mm)');
figure (3);
surf(workspaceX,workspaceY,catenary_errorY);
haxe = gca();
set(haxe,"dataaspectratio", [200 200 0.05]);
title('y error when omiting kynematics details (mm)');
