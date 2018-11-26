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

# Some variables are named in french. You may want to google translate :-)

## Created: 2018-11-01

function [longueur_parabole_suspendue_gauche,longueur_parabole_suspendue_droite, angle_gauche, angle_droit, leftChainT, rightChainT] = computeParabolaWeightChains (targetSledX, targetSledY, leftChainAttachX, leftChainAttachY, rightChainAttachX, rightChainAttachY, leftChainStraightLength, rightChainStraightLength, chainWeightDensity, sledWeight,chainSagCorrectionFactor);

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
  FxTot = T0Sled + T0LChain + T0RChain ;
  
  ## compute sled weight separation on each side
  FySledRight = calculer_Fyd(sledWeight, rightChainAttachX - leftChainAttachX ,sledX); ## erreur du `a décentrage de left et right X attach.
  FySledLeft = calculer_Fyg(sledWeight, rightChainAttachX - leftChainAttachX ,sledX); ## erreur du `a décentrage de left et right X attach.

  ##calcule les angles
  angle_gauche = atan(FySledLeft/FxTot);
  angle_droit = atan(FySledRight/FxTot);
  
  ## chain sag calculation using parabola approximation
  ## sag changes sled position. But we won't account for it here because the parabola 
  ## approximation does not provide information about sled point of equilibrium
  if (chainSagCorrectionFactor>=0)
    longueur_parabole_suspendue_gauche = leftChainStraightLength * (1 + ((chainSagCorrectionFactor / 1E12) * power(cos(angle_gauche),2)  * power(leftChainStraightLength,2)  * power((tan(angle_droit) * cos(angle_gauche))  + sin(angle_gauche),2)));
    longueur_parabole_suspendue_droite = rightChainStraightLength * (1 + ((chainSagCorrectionFactor / 1E12) * power(cos(angle_droit),2) * power(rightChainStraightLength,2) * power((tan(angle_gauche)  * cos(angle_droit)) + sin(angle_droit),2)));
  endif
  
  #assign results

  largeur_entre_points = ChainAttachpointsSeparation;
  leftChainT = sqrt(power(FxTot,2)+power(FySledLeft,2));
  rightChainT = sqrt(power(FxTot,2)+power(FySledRight,2));
endfunction
