// OrangeRx R615X receiver has 6 channels, and one PPM channel which is a combination of all the PPM signals
// into one output.  This program reads the PPM pin and displays the values of the channels.

const int NumberOfChannels = 6;          
const byte InputPin = 21;
const byte TriggerPin = 13;
const int FrameSpace = 8000;

volatile long Current_Time;
volatile long Last_Spike;

volatile byte Current_Channel = 0;
volatile int Spike_Length[NumberOfChannels + 1];
volatile int LastChannelFlag = false;

// Any time a rising edge is seen on pin 2, Spike() is called
// Set up with 8 DATA bits, No PARITY checking, 1 STOP bit
// Set baud rate to 57600 baud
void setup()
{
  delay(100);
  pinMode(TriggerPin, OUTPUT);
  attachInterrupt( 2, Spike, RISING); // Arduino interrupt pin 2 is digital pin 21
  Last_Spike = micros(); // Returns the number of microseconds since the Arduino began the program
  Serial.begin(57600, SERIAL_8N1);
}

//  If all channels are read, get the time for last spike and then display all spikes onto serial port.
void loop()
{

  

  if (LastChannelFlag == true)   
  {
    LastChannel();               
    Display();
  }
}


//  Get the time on a rising edge.
//  If the channel is the final channel, set flag for displaying information.
void Spike()
{
  Current_Time = micros();
  Spike_Length[Current_Channel] = Current_Time - Last_Spike;
  if (Spike_Length[Current_Channel] > FrameSpace) Current_Channel = 0;

  Last_Spike = Current_Time;
  Current_Channel = Current_Channel + 1;

  if (Current_Channel == NumberOfChannels)
  {
    LastChannelFlag = true;
  }
}


//  Reset the LastChannelFlag, Current_Channel counter, and set the Last_Spike to the current time.
//  This will then call the Display() function from the loop.
void LastChannel()
{
  while(digitalRead(InputPin) == HIGH);   
  Current_Time = micros();
  Spike_Length[Current_Channel] = Current_Time - Last_Spike;    
  Current_Channel = 0;
  Last_Spike = Current_Time;  
  LastChannelFlag = false; 
}





// Prints values to Serial Port
// Delimit channel data with '\t' and end all channels with a '\n'
void Display()
{
  
  for ( byte x = 1; x <= 6; x++)
  {
    Serial.print(map(Spike_Length[x], 1000, 2000, 0, 100));
    Serial.print("\t");
  }
  Serial.print("\n");
  Trigger();
}

void Trigger()
{
  if ( map( Spike_Length[1], 1000, 2000, 0, 100 ) > 50 )
  {
    digitalWrite(TriggerPin, HIGH);
  }
  else
  {
    digitalWrite(TriggerPin, LOW);
  }
}
