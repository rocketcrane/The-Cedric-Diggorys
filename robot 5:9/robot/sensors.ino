//-----------------SENSOR FUNCTIONS--------------------


//From HSV source----------------------
// h = [0,360], s = [0,1], v = [0,1]
// if s == 0, then h = -1 (undefined)
void RGBtoHSV( float r, float g, float b, float *h, float *s, float *v )
{
  r = r / 255;
  g = g / 255;
  b = b / 255;
  float min, max, delta;

  min = MIN( r, g, b );
  max = MAX( r, g, b );
  *v = max;       // v

  delta = max - min;

  if ( max != 0 )
    *s = delta / max;   // s
  else {
    // r = g = b = 0    // s = 0, v is undefined
    *s = 0;
    *h = -1;
    return;
  }

  if ( r == max )
    *h = ( g - b ) / delta;   // between yellow & magenta
  else if ( g == max )
    *h = 2 + ( b - r ) / delta; // between cyan & yellow
  else
    *h = 4 + ( r - g ) / delta; // between magenta & cyan

  *h *= 60;       // degrees
  if ( *h < 0 )
    *h += 360;
}
//end HSV source---------------------

float MIN(float a, float b, float c) {
  float currMin = a;
  if (b < currMin) currMin = b;
  if (c < currMin) currMin = c;
  return currMin;
}

float MAX(float a, float b, float c) {
  float currMax = a;
  if (b > currMax) currMax = b;
  if (c > currMax) currMax = c;
  return currMax;
}

float getIrVal(int pin) {
  //return sensor.value * sensor.modifier;
  // return 37-sensor.value;
  return 13 * pow(analogRead(pin) * 0.0048828125, -1);
}

float getSonarVal(int pin) {
  return analogRead(pin) / 5;
}

float getLightVal(int pin) {
  return pow(10, analogRead(pin) * 5.0 / 1024);
}

float getVoltage() {
  return analogRead(VOLT_PIN) / 121.79;
}
