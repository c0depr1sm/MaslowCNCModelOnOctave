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

## Usage: Like the MAslow firmware, call the function with known chain lengths and 
## an estimate initial x,y position, and you'll get an optimised x,y coordinate for 
## these chain length.
## for the "type" parameter, refer to the inverse.m function
##
function [xpos,ypos] = ForwardKynematics (chainALength, chainBLength, xguess, yguess,type)
 KINEMATICSMAXGUESS = 200;
 lengthmargin = 0.02;
 guesscount = 0;
 do 
  [guessLengthA, guessLengthB] = inverse(xguess, yguess, type);
  aChainError = chainALength - guessLengthA;
  bChainError = chainBLength - guessLengthB;
  
  xguess = xguess + 0.1*aChainError - 0.1 *bChainError;
  yguess = yguess - 0.1*aChainError - 0.1 *bChainError;
  guesscount = guesscount +1;
  if((abs(aChainError) < lengthmargin && abs(bChainError) < lengthmargin) || guesscount > KINEMATICSMAXGUESS || guessLengthA > 4000 || guessLengthB > 4000)
   if ((guesscount > KINEMATICSMAXGUESS) || guessLengthA > 4000 || guessLengthB > 4000)
    xpos = 0;
    ypos = 0;
   else
    xpos = xguess;
    ypos = yguess;
   endif
   break
  endif
 until (0==1)
endfunction
