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

function retval = calculer_Fx (totalWeight, leftXSupport, rightXSupport,sledX,sledY)
ratio = (rightXSupport-leftXSupport)/2;
retval = totalWeight*(ratio^2-(sledX-((rightXSupport+leftXSupport)/2))^2)/(2*sledY*ratio); 
endfunction
