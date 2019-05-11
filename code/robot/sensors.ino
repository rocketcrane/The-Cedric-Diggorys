//-----------------SENSOR FUNCTIONS--------------------


//from HSV source -------------------
// h = [0,360], s = [0,1], v = [0,1]
//    if s == 0, then h = -1 (undefined)
void RGBtoHSV( float r, float g, float b, float *h, float *s, float *v )
{
  r=r/255;
  g=g/255;
  b=b/255;
  float min, max, delta;

  min = MIN( r, g, b );
  max = MAX( r, g, b );
  *v = max;       // v

  delta = max - min;

  if( max != 0 )
    *s = delta / max;   // s
  else {
    // r = g = b = 0    // s = 0, v is undefined
    *s = 0;
    *h = -1;
    return;
  }

  if( r == max )
    *h = ( g - b ) / delta;   // between yellow & magenta
  else if( g == max )
    *h = 2 + ( b - r ) / delta; // between cyan & yellow
  else
    *h = 4 + ( r - g ) / delta; // between magenta & cyan

  *h *= 60;       // degrees
  if( *h < 0 )
    *h += 360;

}

//end HSV source--------------------------

float MIN(float a, float b, float c){
  float currMin = a;
  if(b<currMin) currMin = b;
  if(c<currMin) currMin = c;
  return currMin;
}
float MAX(float a, float b, float c){
  float currMax = a;
  if(b>currMax) currMax = b;
  if(c> currMax) currMax = c;
  return currMax;
}

bool isRed(float hue){
  return hue < 363 && hue > 353;
}
bool isBlue(float hue){
  return hue < 202 && hue > 192;
}
bool isYellow(float hue){
  return hue < 87 && hue > 77;
}
bool isGray(float hue){
  return hue < 165 && hue > 150;
}

