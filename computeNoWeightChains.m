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

## Created: 2018-10-01

## This function assumes a coordinate system where y is positive going upward, x going right.
## x = 0 at the horizontal center between left and right gearboxes shafts
## Similar to the Maslow firmware.
## However it assumes the chain from the sled passes UNDER the sproccket.

## Inputs
## sprocketRadius is the avarage sprocket radius (mm) (ignoring cordal effect)
## sledX,sledY are the target sled x and y position (mm) . The sled position is actually the router bit tip center position.
## distBetweenLRMotorsGearBoxShafts (mm) is the distance between left and right gearboxes shafts on a perfectly rigid frame.
## sledRotationDiskRadius (mm) is the same as the maslow cnc firmware
## maxTopBeamTipFlexAndTwist (mm) is the vertical displacement (positive value only) of the gearbox shaft when it holds the whole sled. assuming symetric response of both top beam ends.
## leftChainLengthCorrection (ratio) is about the real average left chain length compared to nominal
## rightChainLengthCorrection (ratio) is about the real average right chain length compared to nominal
## IdealShaftY (mm) is the height of the gearbox shaft over the y origin used by sledY

## Outputs
## leftChainExtent, rightChainExtent are the almost ideal chains lenght on a ideal sprocket and a non-deal top beam. 
## leftChainStraightSection,rightChainStraightSection are the straight chain lenghts sections accounting for chain lenght tolerance, but not sag, nor sled rotation radius
## leftChainX, leftChainY, rightChainX, rightChainY are the chain contact point coordinates for realy straight chains

## The gearbox shaft holding the chain sprocket, and the router bit tip center position are the objects of this computation.

function [leftChainExtent, rightChainExtent, leftChainStraightSection,rightChainStraightSection,leftChainAngle, rightChainAngle, leftChainX, leftChainY, rightChainX, rightChainY] = computeNoWeightChains (sprocketRadius, sledX,sledY, distBetweenLRMotorsGearBoxShafts, maxTopBeamTipFlexAndTwist, idealShaftY)
 ## estimate initial gearbox shaft positions
 leftShaftY = idealShaftY;
 rightShaftY = idealShaftY;
 leftShaftX = -distBetweenLRMotorsGearBoxShafts/2;
 rightShaftX = distBetweenLRMotorsGearBoxShafts/2;
 
 ## compute vertical beam Tip deflection due to sled weight, according to sled horizontal position
 ## The top beam tip vertical deflection is assumed to be a function of the vertical load it carries.
 ## Assumption: leftShaftX = -rightShaftX
 distanceRatio = (distBetweenLRMotorsGearBoxShafts/2 - sledX)/ distBetweenLRMotorsGearBoxShafts;
 topBeamLeftTipFlexAndTwistVerticalCorrection = (distanceRatio^2 * maxTopBeamTipFlexAndTwist);
 topBeamRightTipFlexAndTwistVerticalCorrection = ((1.0-distanceRatio)^2 * maxTopBeamTipFlexAndTwist);
 
 ## Compute the new shafts Y positions 
 adjustedLeftShaftY = leftShaftY - topBeamLeftTipFlexAndTwistVerticalCorrection;
 adjustedRightShaftY = rightShaftY - topBeamRightTipFlexAndTwistVerticalCorrection;
 
 ## compute gearbox shafts distances to the router bit tip (also known as the sled x,y position)
 leftShaftDistance = sqrt((leftShaftX - sledX)^2+(adjustedLeftShaftY - sledY)^2);
 rightShaftDistance = sqrt((rightShaftX - sledX)^2+(adjustedRightShaftY - sledY)^2);
 
 ## compute the chain angles relative to horizontal. 
 ## considers the effect of the sprocket average radius
 ## We assume here the chain from the sled passes UNDER the sproccket.
 leftChainAngle  = asin((adjustedLeftShaftY  - sledY)/leftShaftDistance)  - asin(sprocketRadius/leftShaftDistance);
 rightChainAngle = asin((adjustedRightShaftY - sledY)/rightShaftDistance) - asin(sprocketRadius/rightShaftDistance);
 
 ## compute coordinates of the chains contact points with the sprockets
 leftChainX = leftShaftX - sprocketRadius*sin(leftChainAngle);
 rightChainX = rightShaftX + sprocketRadius*sin(rightChainAngle);
 
 leftChainY = adjustedLeftShaftY - sprocketRadius*cos(leftChainAngle);
 rightChainY = adjustedRightShaftY - sprocketRadius*cos(rightChainAngle);
 
 ## compute amount of chain wrapped from sprocket top to contact point 
 leftChainAroundSprocket  = sprocketRadius * (pi - leftChainAngle);
 rightChainAroundSprocket = sprocketRadius * (pi - rightChainAngle);
 
 ## Compute the straight chain length from the sprocket contact point to the bit (sled)
 leftChainStraightSection = sqrt((leftShaftDistance)^2-(sprocketRadius)^2);
 rightChainStraightSection = sqrt((rightShaftDistance)^2-(sprocketRadius)^2);
 
 ## This function does not account for chain sag
 ## does not account for chain stretch nor link lenght ratio error
 ## and does not remove sled rotation radius
 
 leftChainExtent  = leftChainAroundSprocket + leftChainStraightSection; 
 rightChainExtent = rightChainAroundSprocket + rightChainStraightSection;
 
endfunction
