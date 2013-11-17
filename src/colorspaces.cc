/*
 * colorspaces.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation; see colorspaces.hh.
 */

#include <algorithm>
#include "colorspaces.hh"

using namespace std;

namespace textured_localization
{

void rgb_to_hsv( int r, int g, int b, double& h, double& s, double& v )
{
  double fr = r/255.0; 
  double fg = g/255.0; 
  double fb = b/255.0;
  int imax = max( max( r, g ), b );
  int imin = min( min( r, g ), b );
  double fmax = imax/255.0 ;
  double fmin = imin/255.0 ;
  double multiplier = ( imin == imax ) ? 0.0 : 60/( fmax - fmin ) ;

  if( r == imax ) 	// red is dominant
  {
    h = ( multiplier*( fg - fb ) + 360 ) ;
    while ( h >= 360 ) h -= 360 ;	// take quick modulus, % doesn't work with floats
  }
  else if( g == imax )// green is dominant
    h = multiplier*( fb - fr ) + 120 ;
  else				// blue is dominant
    h = multiplier*( fr - fg ) + 240 ;	
  if( imax == 0 )
    s = 0 ;
  else
    s = 1 - ( fmin/fmax ) ;
  v = fmax;
}

}
