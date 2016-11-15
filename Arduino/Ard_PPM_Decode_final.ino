//OrangeRx R615X receiver has 6 channels, and one PPM channel which is a combination of all the PPM signals
//into one output.  This program reads the PPM pin and displays the values of the channels.

const int NumberOfChannels = 6;          
const byte InputPin = 21;                
const int FrameSpace = 8000;

volatile long Current_Time;
volatile long Last_Spike;

volatile byte Current_Channel = 0;
volatile int Spike_Length[NumberOfChannels + 1];
volatile int LastChannelFlag = false;

void setup()
{
  delay(100);
  attachInterrupt( 2, Spike, RISING); // Arduino interrupt pin 2 is digital pin 21
  Last_Spike = micros(); // Returns the number of microseconds since the Arduino began the program
  Serial.begin(57600, SERIAL_8N1);
  
  // Set up with 8 DATA bits, No PARITY checking, 1 STOP bit
  // Set baud rate to 57600 baud
}


void loop()
{

  if (LastChannelFlag == true)   
  {
    LastChannel();               
    Display();
  }
}  //end loop()




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



void LastChannel()
{
  while(digitalRead(InputPin) == HIGH);   
  Current_Time = micros();
  Spike_Length[Current_Channel] = Current_Time - Last_Spike;    
  Current_Channel = 0;
  Last_Spike = Current_Time;  
  LastChannelFlag = false; 
}





// Prints values to Serial Monitor
void Display()
{
  
  for ( byte x = 1; x <= 6; x++)
  {
    Serial.print(map(Spike_Length[x], 1000, 2000, 0, 100));
    Serial.print("\t");
  }
  Serial.print("\n");
}

