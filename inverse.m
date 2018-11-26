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


## Usage: 
## type = 
## "bare" for simple sprocket triangular.
## "flex" for simple triangular including beam flex
## "allparabola" best estimation with the parabola sag approximation.
## "allcatenary" most realistic estimation with the catenary equation.
## "allcatenary_with_error" most realistic estimation with the catenary equation and parametric error induction.

function   [LengthA, LengthB] = inverse(xpos, ypos,type)
  ## earth location parameters
  accG = 9.8; ## m/s^2
  
  ## Maslow cnc parameters
  workspaceHeight = 1222; ## mm
  workspaceWidth = 1240; ## mm
  
  ## My 11.5 feet wide maslow
  ##distanceBetweenLRShafts = 3501.2; ## mm 
  ##workspaceTopToShaftsHeight = 617; ## mm // My 11.5 feet wide maslow
  ##maxTopBeamTipFlexAndTwist = 2.9; ## mm
  ##sledRotationRadius = 141.75; ## mm
  ##sledYOffsetAboveBit = 0;# -0.25; ## mm
  ##sledXOffsetRightOfBit = 0;# -0.75; ## mm
  ##leftChainLengthCorrectionRatio = 1; ##.0013; ## ratio
  ##rightChainLengthCorrectionRatio = 1; ## ratio
  ##sledMass = 11.6;
  ##SprocketRadius = 6.35*10/(2*pi); ## mm
  ##chainSagCorrectionFactor = 6.95; # 5.8; ##matches the catenary
  
  ## 10 feet Stock maslow cnc
  distanceBetweenLRShafts = 2978.4; ## mm 
  workspaceTopToShaftsHeight = 463.0; ## // Stock maslow cnc
  maxTopBeamTipFlexAndTwist = 2.9; ## mm // assume a rigid beam for now...
  sledRotationRadius = 141.75; ## mm
  sledYOffsetAboveBit = 0.0; ## mm
  sledXOffsetRightOfBit = 0.0; ## mm
  leftChainLengthCorrectionRatio = 1; ##.0013; ## ratio
  rightChainLengthCorrectionRatio = 1; ## ratio
  sledMass = 11.6;
  SprocketRadius = 6.35*10/(2*pi); ## mm
  ## parabola approximation factor when approximating sag with a parabola
  chainSagCorrectionFactor = 7.6; # 5.8; ## when not considering chain weight in tension
  
  motorX = distanceBetweenLRShafts/2;
  sledWeight = sledMass * accG;
  chainDensity = 14.06E-2; ## kg/m, use 1E-6; # for lightweight chain
  chainWeightDensity = chainDensity * accG;
  ## <https://tt-net.tsubakimoto.co.jp/tecs/pdct/cdc/pdct_Dtl_CRC.asp?kata=RS25-1>
  chainElongationFactor = 4.1E-6; ## fraction added / N 2.6mm / 0.640kN /1000 mm/m
  
  ## Model parameters
  originY = workspaceHeight/2 +workspaceTopToShaftsHeight; ## above Maslow CNC Y origin
  originX = 0; ## mm
  
  ## Compensations parameters to try to correct position within existing Maslow settings
  cheatLRDistance = 0; #2.5; ##(mm) reduction to straighten the top bow
  cheatLRMotorsYOffsetAboveWorkSurface = 0; #2.5; ##(mm) increase to raise beam too high
  cheatRotationRadius = 0; #2.5; ##(mm) reduction to have a short radius
  cheatLeftChainLength = 0; #2.5; ##(mm) reduction to have a short chain
  cheatChainDensity = 1E-6; # 14.06E-2; # 1E-6; # kg/m is the integral value used in the inverse calculation
  cheatChainWeightDensity = cheatChainDensity * accG;
  cheatChainElongationFactor = 0; #4.1E-6; # kg/m is the integral value used in the inverse calculation
  cheatleftChainLengthCorrectionRatio = 0; #0.0013; ## adds to leftChainLengthCorrectionRatio
  cheatmaxTopBeamTipFlexAndTwist = 0; # mm added to the beam flex maxTopBeamTipFlexAndTwist
  
  ## solving parameters (for catenary calculation only)
  maxSagError = 0.01; ##(mm)
  maxWidthError = 0.01; ##(mm)
  maxIteration = 8; ## optimisation refinment for a given sled position
  
  sledX = xpos;
  sledY = ypos;
  
  switch (type)
  case "bare"
    ## initial geometry and chains lenght estimation without sag -> call first to gete chain angles estimations
    [leftChainExtent_raw, rightChainExtent_raw, leftChainStraightLength_raw,rightChainStraightLength_raw, leftStraightChainAngle, rightStraightChainAngle, leftChainAttachX, leftChainAttachY, rightChainAttachX, rightChainAttachY] = computeNoWeightChains (SprocketRadius , sledX, sledY, distanceBetweenLRShafts,0, originY);
    
    ## get raw length before triangular link radius 
    LengthA = leftChainExtent_raw;
    LengthB = rightChainExtent_raw;
  case "flex"
    ## initial geometry and chains lenght estimation without sag -> call first to gete chain angles estimations
    [leftChainExtent_raw, rightChainExtent_raw, leftChainStraightLength_raw,rightChainStraightLength_raw, leftStraightChainAngle, rightStraightChainAngle, leftChainAttachX, leftChainAttachY, rightChainAttachX, rightChainAttachY] = computeNoWeightChains (SprocketRadius , sledX, sledY, distanceBetweenLRShafts,maxTopBeamTipFlexAndTwist, originY);
    
    ## get raw length before triangular link radius 
    LengthA = leftChainExtent_raw;
    LengthB = rightChainExtent_raw;
    
  case "allparabola"
    ## initial geometry and chains lenght estimation without sag -> call first to gete chain angles estimations
    [leftChainExtent_raw, rightChainExtent_raw, leftChainStraightLength_raw,rightChainStraightLength_raw, leftStraightChainAngle, rightStraightChainAngle, leftChainAttachX, leftChainAttachY, rightChainAttachX, rightChainAttachY] = computeNoWeightChains (SprocketRadius , sledX, sledY, distanceBetweenLRShafts,maxTopBeamTipFlexAndTwist, originY);
    
    ## consider triangular link out-of-center error, 
    sledX = sledX - sledXOffsetRightOfBit;
    sledY = sledY - sledYOffsetAboveBit;
    
    ## second geometry and chains lenght estimation without sag -> call to get impact of sled offcenter ccorrections
    [leftChainExtent, rightChainExtent, leftChainStraightLength,rightChainStraightLength, leftStraightChainAngle, rightStraightChainAngle, leftChainAttachX, leftChainAttachY, rightChainAttachX, rightChainAttachY] = computeNoWeightChains (SprocketRadius , sledX, sledY, distanceBetweenLRShafts,maxTopBeamTipFlexAndTwist, originY);
    
    ## id chain segment wrappd around sprocket.
    leftChainAroundSprocket = leftChainExtent - leftChainStraightLength;
    rightChainAroundSprocket = rightChainExtent - rightChainStraightLength;
    
    ## change coordinate system for sag calculation 
    targetSledX = sledX;
    targetSledY = originY-sledY; ## mm
    leftChainAttachY = originY - leftChainAttachY;
    rightChainAttachY = originY - rightChainAttachY;
    
    ## establish the horizontal chain attach points distance. This is a target for the sag calculations.
    ChainAttachpointsSeparation = rightChainAttachX - leftChainAttachX;
 
    [longueur_parabole_suspendue_gauche,longueur_parabole_suspendue_droite, angle_gauche, angle_droit, leftChainT, rightChainT] = computeParabolaWeightChains (targetSledX, targetSledY,leftChainAttachX, leftChainAttachY, rightChainAttachX, rightChainAttachY,  leftChainStraightLength, rightChainStraightLength, chainWeightDensity, sledWeight, chainSagCorrectionFactor);
    #longueur_ideale_suspendue_gauche = 1;
    #longueur_ideale_suspendue_droite = 1;
    #largeur_entre_points = ChainAttachpointsSeparation;
    #angle_gauche = pi/3;
    #angle_droit = pi/3;
    #leftChainT = 10;
    #rightChainT = 10;
    
    ## adapt chain length to fit calibration parameters
    ## include chain stretch and pitch error
    leftChainElongationFactor = 1+ (leftChainT * chainElongationFactor);
    leftChainSuspendedSegmentLength = longueur_parabole_suspendue_gauche / (leftChainElongationFactor * leftChainLengthCorrectionRatio);
    
    rightChainElongationFactor = 1+ (rightChainT * chainElongationFactor);
    rightChainSuspendedSegmentLength = longueur_parabole_suspendue_droite / (rightChainElongationFactor * rightChainLengthCorrectionRatio);
    
    ## Final chain extent to feed on Maslow cnc is the combination of relaxed and tensionned segments, and removed sled rotation radius segment 
    leftChainExtent  = leftChainAroundSprocket + leftChainSuspendedSegmentLength; 
    leftChainExtentOnTriangularLink = leftChainExtent - sledRotationRadius;
    rightChainExtent = rightChainAroundSprocket + rightChainSuspendedSegmentLength;
    rightChainExtentOnTriangularLink = rightChainExtent - sledRotationRadius;
    
    ## get raw length before triangular link radius 
    LengthA = leftChainExtent;
    LengthB = rightChainExtent;
    
  case "allcatenary"
    ## initial geometry and chains lenght estimation without sag -> call first to gete chain angles estimations
    [leftChainExtent_raw, rightChainExtent_raw, leftChainStraightLength_raw,rightChainStraightLength_raw, leftStraightChainAngle, rightStraightChainAngle, leftChainAttachX, leftChainAttachY, rightChainAttachX, rightChainAttachY] = computeNoWeightChains (SprocketRadius , sledX, sledY, distanceBetweenLRShafts,maxTopBeamTipFlexAndTwist, originY);
    
    ## consider triangular link out-of-center error, 
    sledX = sledX - sledXOffsetRightOfBit;
    sledY = sledY - sledYOffsetAboveBit;
    
    ## second geometry and chains lenght estimation without sag -> call to get impact of sled offcenter ccorrections
    [leftChainExtent, rightChainExtent, leftChainStraightLength,rightChainStraightLength, leftStraightChainAngle, rightStraightChainAngle, leftChainAttachX, leftChainAttachY, rightChainAttachX, rightChainAttachY] = computeNoWeightChains (SprocketRadius , sledX, sledY, distanceBetweenLRShafts,maxTopBeamTipFlexAndTwist, originY);
    
    ## id chain segment wrappd around sprocket.
    leftChainAroundSprocket = leftChainExtent - leftChainStraightLength;
    rightChainAroundSprocket = rightChainExtent - rightChainStraightLength;
    
    ## change coordinate system for sag calculation 
    targetSledX = sledX;
    targetSledY = originY-sledY; ## mm
    leftChainAttachY = originY - leftChainAttachY;
    rightChainAttachY = originY - rightChainAttachY;
    
    ## establish the horizontal chain attach points distance. This is a target for the sag calculations.
    ChainAttachpointsSeparation = rightChainAttachX - leftChainAttachX;
    
    ## lauch the chain sag calculation for that sled coordinate
    [longueur_ideale_suspendue_gauche,longueur_ideale_suspendue_droite, leftChainAttachY, rightChainAttachY, largeur_entre_points, angle_gauche, angle_droit, angleSledGauche, angleSledDroit, leftChainT, rightChainT, catenary_a, erreur_sledX, iteration] = computeRealWeightChains (targetSledX, targetSledY, leftChainAttachX, leftChainAttachY, rightChainAttachX, rightChainAttachY, leftChainStraightLength, rightChainStraightLength, chainWeightDensity, sledWeight, maxSagError, maxWidthError, maxIteration);
    #longueur_ideale_suspendue_gauche = 1;
    #longueur_ideale_suspendue_droite = 1;
    #largeur_entre_points = ChainAttachpointsSeparation;
    #angle_gauche = pi/3;
    #angle_droit = pi/3;
    #leftChainT = 10;
    #rightChainT = 10;
    #catenary_a = 2000;
    #erreur_sledX = 1;
    
    ## adapt chain length to fit calibration parameters
    ## include chain stretch and pitch error
    leftChainElongationFactor = 1+ (leftChainT * chainElongationFactor);
    leftChainSuspendedSegmentLength = longueur_ideale_suspendue_gauche / (leftChainElongationFactor * leftChainLengthCorrectionRatio);
    
    rightChainElongationFactor = 1+ (rightChainT * chainElongationFactor);
    rightChainSuspendedSegmentLength = longueur_ideale_suspendue_droite / (rightChainElongationFactor * rightChainLengthCorrectionRatio);
    
    ## Final chain extent to feed on Maslow cnc is the combination of relaxed and tensionned segments, and removed sled rotation radius segment 
    leftChainExtent  = leftChainAroundSprocket + leftChainSuspendedSegmentLength; 
    leftChainExtentOnTriangularLink = leftChainExtent - sledRotationRadius;
    rightChainExtent = rightChainAroundSprocket + rightChainSuspendedSegmentLength;
    rightChainExtentOnTriangularLink = rightChainExtent - sledRotationRadius;
    
    ## get raw length before triangular link radius 
    LengthA = leftChainExtent;
    LengthB = rightChainExtent;
  case "allcatenary_with_error"
    # use distanceBetweenLRShafts-cheatLRDistance
    
    ## initial geometry and chains lenght estimation without sag -> call first to gete chain angles estimations
    [leftChainExtent_raw, rightChainExtent_raw, leftChainStraightLength_raw,rightChainStraightLength_raw, leftStraightChainAngle, rightStraightChainAngle, leftChainAttachX, leftChainAttachY, rightChainAttachX, rightChainAttachY] = computeNoWeightChains (SprocketRadius , sledX, sledY, distanceBetweenLRShafts-cheatLRDistance,maxTopBeamTipFlexAndTwist+cheatmaxTopBeamTipFlexAndTwist, originY+cheatLRMotorsYOffsetAboveWorkSurface);
    
    ## consider triangular link out-of-center error, 
    sledX = sledX - sledXOffsetRightOfBit;
    sledY = sledY - sledYOffsetAboveBit;
    
    ## second geometry and chains lenght estimation without sag -> call to get impact of sled offcenter ccorrections
    [leftChainExtent, rightChainExtent, leftChainStraightLength,rightChainStraightLength, leftStraightChainAngle, rightStraightChainAngle, leftChainAttachX, leftChainAttachY, rightChainAttachX, rightChainAttachY] = computeNoWeightChains (SprocketRadius , sledX, sledY, distanceBetweenLRShafts-cheatLRDistance,maxTopBeamTipFlexAndTwist+cheatmaxTopBeamTipFlexAndTwist, originY+cheatLRMotorsYOffsetAboveWorkSurface);
    
    ## id chain segment wrappd around sprocket.
    leftChainAroundSprocket = leftChainExtent - leftChainStraightLength;
    rightChainAroundSprocket = rightChainExtent - rightChainStraightLength;
    
    ## change coordinate system for sag calculation 
    targetSledX = sledX;
    targetSledY = originY+cheatLRMotorsYOffsetAboveWorkSurface-sledY; ## mm
    leftChainAttachY = originY+cheatLRMotorsYOffsetAboveWorkSurface - leftChainAttachY;
    rightChainAttachY = originY+cheatLRMotorsYOffsetAboveWorkSurface - rightChainAttachY;
    
    ## establish the horizontal chain attach points distance. This is a target for the sag calculations.
    ChainAttachpointsSeparation = rightChainAttachX - leftChainAttachX;
    
    ## lauch the chain sag calculation for that sled coordinate
    [longueur_ideale_suspendue_gauche,longueur_ideale_suspendue_droite, leftChainAttachY, rightChainAttachY, largeur_entre_points, angle_gauche, angle_droit, angleSledGauche, angleSledDroit, leftChainT, rightChainT, catenary_a, erreur_sledX, iteration] = computeRealWeightChains (targetSledX, targetSledY, leftChainAttachX, leftChainAttachY, rightChainAttachX, rightChainAttachY, leftChainStraightLength, rightChainStraightLength, cheatChainWeightDensity, sledWeight, maxSagError, maxWidthError, maxIteration);
    #longueur_ideale_suspendue_gauche = 1;
    #longueur_ideale_suspendue_droite = 1;
    #largeur_entre_points = ChainAttachpointsSeparation;
    #angle_gauche = pi/3;
    #angle_droit = pi/3;
    #leftChainT = 10;
    #rightChainT = 10;
    #catenary_a = 2000;
    #erreur_sledX = 1;
    
    ## adapt chain length to fit calibration parameters
    ## include chain stretch and pitch error
    leftChainElongationFactor = 1+ (leftChainT * cheatChainElongationFactor);
    leftChainSuspendedSegmentLength = longueur_ideale_suspendue_gauche / (leftChainElongationFactor * (leftChainLengthCorrectionRatio+cheatleftChainLengthCorrectionRatio));
    
    rightChainElongationFactor = 1+ (rightChainT * cheatChainElongationFactor);
    rightChainSuspendedSegmentLength = longueur_ideale_suspendue_droite / (rightChainElongationFactor * rightChainLengthCorrectionRatio);
    
    ## Final chain extent to feed on Maslow cnc is the combination of relaxed and tensionned segments, and removed sled rotation radius segment 
    leftChainExtent  = leftChainAroundSprocket + leftChainSuspendedSegmentLength; 
    leftChainExtentOnTriangularLink = leftChainExtent - sledRotationRadius;
    rightChainExtent = rightChainAroundSprocket + rightChainSuspendedSegmentLength;
    rightChainExtentOnTriangularLink = rightChainExtent - sledRotationRadius;
    
    ## get raw length before triangular link radius 
    LengthA = leftChainExtent-cheatRotationRadius+cheatLeftChainLength;
    LengthB = rightChainExtent-cheatRotationRadius;
  endswitch
  
endfunction
