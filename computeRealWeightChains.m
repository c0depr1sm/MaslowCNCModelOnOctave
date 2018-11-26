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

# Some variables are named in french. You might want to google translate :-)

## Created: 2018-10-01

## This function assumes a coordinate system where y is positive going downward, x going righ.
## x = 0 at the horizontal center between left and right gearboxes shafts
## y = 0 at the **ideal horizontal line** between gearbox shafts on a perfectly rigid top beam
## (actually, the catenary computations will search different origins for the catenary vertex of the left and right chains.)
## NOT similar to the Maslow firmware.
## It assumes the chain from the sled passes UNDER the sproccket.

## Inputs
## targetSledX, targetSledY, the resulting sled position to be acheived once sag adjustment is included
## leftChainAttachX, leftChainAttachY, the coordinates of the chain contact points on sprockets
## rightChainAttachX, rightChainAttachY, 
## leftChainStraightLength, rightChainStraightLength, the ideal weigth less chain straight segment  
## chainWeightDensity, 
## sledWeight, 
## maxSagError,
## maxWidthError 
## maxIteration -> must be greater than 2

## Outputs
## longueur_gauche, longueur_droite,
## longueur_ideale_suspendue_gauche,longueur_ideale_suspendue_droite, 
## leftChainAttachY, rightChainAttachY, 
## largeur_entre_points, 
## angle_gauche, angle_droit, 
## leftChainT, rightChainT, 
## catenary_a, 
## erreur_sledX

function [longueur_ideale_suspendue_gauche,longueur_ideale_suspendue_droite, leftChainAttachY, rightChainAttachY, largeur_entre_points, angle_gauche, angle_droit, angleSledGauche, angleSledDroit, leftChainT, rightChainT, catenary_a, erreur_sledX, iteration] = computeRealWeightChains (targetSledX, targetSledY, leftChainAttachX, leftChainAttachY, rightChainAttachX, rightChainAttachY, leftChainStraightLength, rightChainStraightLength, chainWeightDensity, sledWeight, maxSagError, maxWidthError, maxIteration)

## initial sled position is the target
sledX = targetSledX ;

## ici on a des coordonnées x et y relatives au top beam non-fléchi pour chaque bout de chaine.
## et on a des longueurs de chaines idéales droites.
leftChainInitialWeightEstimate = chainWeightDensity * leftChainStraightLength/1000;
leftChainCoGX = (leftChainAttachX + sledX)/2;
leftChainCoGY = (leftChainAttachY + targetSledY)/2;
rightChainInitialWeightEstimate = chainWeightDensity * rightChainStraightLength/1000;
rightChainCoGX = (rightChainAttachX + sledX)/2;
rightChainCoGY = (rightChainAttachY + targetSledY)/2;
ChainAttachpointsSeparation = rightChainAttachX - leftChainAttachX;

## Initial horizontal (T0) tension estimation 
## each of the sled (with rollers), left chain and right chain masses contributte to toal tension. 
T0Sled = calculer_Fx(sledWeight, leftChainAttachX,rightChainAttachX ,sledX,targetSledY);
T0LChain = calculer_Fx(leftChainInitialWeightEstimate, leftChainAttachX,rightChainAttachX ,leftChainCoGX,leftChainCoGY);
T0RChain = calculer_Fx(rightChainInitialWeightEstimate, leftChainAttachX,rightChainAttachX ,rightChainCoGX,rightChainCoGY);
T0Tot = T0Sled + T0LChain + T0RChain ;
aTot = T0Tot / (chainWeightDensity/1000);

#sag changes sled position. And sled position changes horizontal tension. So we'll iterate to a solution.
iter = 0;
do
  ## compute sled weight separation on each side
  FySledRight = calculer_Fyd(sledWeight, rightChainAttachX - leftChainAttachX ,sledX); ## erreur du `a décentrage de left et right X attach.
  FySledLeft = calculer_Fyg(sledWeight, rightChainAttachX - leftChainAttachX ,sledX); ## erreur du `a décentrage de left et right X attach.
  
  ## compute s sled left and right side (mm)
  sSledLeft = FySledLeft /(chainWeightDensity/1000);
  sSledRight = FySledRight /(chainWeightDensity/1000);
  
  #estimate width of the catenaries hiden into the sled weight
  xSledLeft = aTot*asinh(sSledLeft/aTot);
  xSledRight = aTot*asinh(sSledRight/aTot);
  
  #estimate height of the catenaries hiden into the sled weight
  ## find yl and yr from a and ssledl and ssledr y = sqrt(a^2+s^2)
  ySledLeft = sqrt(aTot^2+sSledLeft^2);
  ySledRight = sqrt(aTot^2+sSledRight^2);
  
  ## find phileft and phi right from y ad a y = asec(phi)
  phiSledLeft = asec(ySledLeft/aTot);
  phiSledRight = asec(ySledRight/aTot);
  
  ## identify target y of each chain contact point from catenary vertex
  yTotLeft = ySledLeft + targetSledY- leftChainAttachY;
  yTotRight = ySledRight + targetSledY- rightChainAttachY;
  
  ## compute s (total sagging lenght) and real chain sagging lentht of each left and right chains
  sTotLeft = sqrt(yTotLeft^2-aTot^2);
  deltaSLeft =  sTotLeft -sSledLeft;
  sTotRight =sqrt(yTotRight^2-aTot^2);
  deltaSRight =  sTotRight -sSledRight;
  
  #observe sled height under top mounting point
  ## this is actually just a sanity check as it does not depend on the sag calculation
  DeltaYLeft = yTotLeft - ySledLeft;
  sledYLeftFromTop = leftChainAttachY +DeltaYLeft;
  DeltaYRight = yTotRight - ySledRight;
  sledYRightFromTop = rightChainAttachY +DeltaYRight;
  
  ## find chain angle at contact point on sprockets
  phiTotLeft = asec(yTotLeft/aTot);
  phiTotRight = asec(yTotRight/aTot);
  
  ## on a catenary, x distance at contact point depends on horizontal tension and lenght
  xTopLeft = aTot*asinh(sTotLeft/aTot);
  xTopRight = aTot*asinh(sTotRight/aTot);
  
  ##Then compute sag estimation error on chain attach points horizontal position
  CatenaryWidth = xTopLeft -xSledLeft + xTopRight -xSledRight ;
  ## compute a resulting sledX estimation if computed sagging chains reach wider than the true distance beween contact points
  resultingSledX = rightChainAttachX-((xTopRight -xSledRight)*(ChainAttachpointsSeparation/CatenaryWidth));
  sledXError = resultingSledX - targetSledX;
  catenaryWidthError = ChainAttachpointsSeparation-CatenaryWidth;
  
  if ((abs (sledXError) <= maxSagError) && (abs(catenaryWidthError) <= maxWidthError) && (iter>2)) 
    break; ## exit do until loop
  endif
  
  ## If the error is too big, then sledX or horizontal tension are not right.
  # let's assume chain angle is accurate since slight changes in sledX or sag should have little effect on chain angle at contact point.
  sledX  = sledX - sledXError;
  ## review the catenary tension estimation
  aTot = aTot *ChainAttachpointsSeparation/CatenaryWidth;
  T0Tot = aTot * (chainWeightDensity/1000);
  iter +=1; ## count iterations
until (iter>=maxIteration) 

#assign results
angle_gauche = phiTotLeft ;
angle_droit = phiTotRight;
angleSledGauche = phiSledLeft;
angleSledDroit = phiSledRight;
largeur_entre_points = CatenaryWidth;
erreur_sledX = sledXError;
longueur_ideale_suspendue_gauche = deltaSLeft;
longueur_ideale_suspendue_droite = deltaSRight;
longueur_gauche = leftChainStraightLength;
longueur_droite = rightChainStraightLength;
catenary_a = aTot;
leftChainT = (chainWeightDensity)*sqrt(aTot^2+sTotLeft^2)/1000;
rightChainT = (chainWeightDensity)*sqrt(aTot^2+sTotRight^2)/1000;
iteration = iter;
endfunction
