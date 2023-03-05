#include <FastLED.h>
#include <time.h>

#define LED_PIN     11
#define NUM_LEDS    27
#define BRIGHTNESS  255
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 100

// Global LED array
CRGB leds[NUM_LEDS];

// Possible animations
enum {GREEN, YELLOW, PURPLE, START, RED, BLUE, OFF};
int animation = START;

// LED loop logistics, mostly timer stuff for flashing without delays
int pos = 0;
bool is_yellow = false;
bool is_purple = false;
int period = 90;
int remTime = 0;
int actTime = 0;

// Vars for calculating loop delta time
int loopStart = 0;
int dt = 0;

void setup() {
  // Register strip with FastLED
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  
  // Serial at 9600 baud
  Serial.begin(9600);
}


void loop()
{
  loopStart = millis();

  // Parse a byte from the Serial bus
  readCommand();

  // Run the command corresponding to the current animation
  // and write the corresponding byte to Serial for debugging
  switch(animation)
  {
    case GREEN:
      GreenLoop();
      Serial.write('g');
      break;
    case YELLOW:
      YellowLoop();
      Serial.write('y');
      break;
    case PURPLE:
      PurpleLoop();
      Serial.write('p');
      break;
    case START:
      StartLoop();
      Serial.write('s');
      break;
    case RED:
      RedLoop();
      Serial.write('r');
      break;
    case BLUE:
      BlueLoop();
      Serial.write('b');
    case OFF:
      OffLoop();
      Serial.write('o');
      break; 
  }

  // Calculate the delta
  dt = (1000 / UPDATES_PER_SECOND) - (loopStart - millis());
  
  // Handle the case if the loop took longer than we wanted
  if (dt > 0) {

    // Delay the loop until the next cycle starts
    delay(dt);
  }
}

void readCommand() 
{
  // Check how many bytes are in the Serial buffer
  byte n = Serial.available();
  
  // If there's at least one byte buffered, start the read
  if(n > 0)
  {
    // Pull the first byte from the buffer and store it
    byte c = Serial.read();
    
    // Run the corresponding command, if there is one
    // If it's yellow or purple we also set up their timers
    switch (c) {
      case 'y':
        is_yellow = false;
        remTime = 0;
        actTime = 0;
        animation = YELLOW; break;
      
      case 'p':
        is_purple = false;
        remTime = 0;
        actTime = 0;
        animation = PURPLE; break;
      
      case 'g':
        animation = GREEN; break;

      case 's':
        pos = 0;
        animation = START; break;
      
      case 'r':
        animation = RED; break;
      
      case 'b':
        animation = BLUE; break;
      
      case 'o':
        animation = OFF; break;
    }
  }
}

void StartLoop() //code for startup (Red LEDs filling ring)
{ 
  FastLED.show();

  // Quick and dirty way to double the length of time to finish the loop
  leds[(int)(pos/2)]= CRGB::Red;
  if (!(pos+1 > (NUM_LEDS * 2))) {
    pos++;
  }
}

void YellowLoop() // Yellow blinking LEDs for cones
{
  // Get the current time
  actTime=millis();

  // Check if it's been long enough since the last flash
  if(actTime - remTime >= period){
    // Reset the timer
    remTime=actTime;

    // Flip state: if the LEDs are currently yellow, turn them black
    // If they're currently black turn them yellow
    if(!is_yellow){
      is_yellow = true;
      fill_solid(leds, NUM_LEDS, CRGB::Yellow);
      FastLED.show();
    }
    else
    {
      is_yellow = false;
      fill_solid(leds,NUM_LEDS, CRGB::Black);
      FastLED.show();
    }
  } 
}

void PurpleLoop() // Purple blinking LEDs for cubes
{
  // Store curr time
  actTime=millis();

  // Check time since last flash
  if(actTime - remTime >= period){
    remTime=actTime;

    // Invert state
    if(!is_purple){
      is_purple = true;
      fill_solid(leds, NUM_LEDS, CRGB::Purple);
      FastLED.show();
    }
    else
    {
      is_purple = false;
      fill_solid(leds,NUM_LEDS, CRGB::Black);
      FastLED.show();
    }
   }
}

void GreenLoop() // Solid green; indicates intake
{
  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
}

void RedLoop() // We use this as idle when on red alliance
{
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
}

void BlueLoop() // Idle state for blue alliance
{
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
}

void OffLoop() // Turn LEDs all black, which is the same thing as off
{ 
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}