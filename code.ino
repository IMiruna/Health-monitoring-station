#include <Wire.h> 
#include<SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#define DHTPIN 2     // Pin for DHT sensor
#define DHTTYPE DHT11   
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

// Data wire is connected to the Arduino digital pin 7
#define ONE_WIRE_BUS 7
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

//Variables DHT
float hum;  //Stores humidity value
float temp; //Stores temperature value


SoftwareSerial Bluetooth(10, 9);  // Pins for Bluetooth module

static const int numberOfSmoothSamples = 3;
static const int BPMtreshold = 5;
static const int maxBPM = 150;

static const int timeBetweenReads = 20;
static const int numberOfSamplesPerBit = 10;

static const int pulsePin = 0;      // Pulse Sensor red wire connected to analog pin 0

volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded!
volatile boolean Pulse = false;     // “True” when User’s live heartbeat is detected. “False” when not a “live beat”.
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

volatile int rate[numberOfSamplesPerBit];                      // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;   // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;    // used to find IBI
volatile int P = 512;                       // used to find peak in pulse wave, seeded
volatile int T = 512;                       // used to find trough in pulse wave, seeded
volatile int thresh = 525;                  // used to find instant moment of heart beat, seeded
volatile int amp = 100;                     // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat = true;          // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = false;        // used to seed rate array so we startup with reasonable BPM

int lastNBPM[numberOfSmoothSamples];
int currentBPMIndex = 0;
int meanBPMValue = 0;

void setup()
{
  Serial.begin(9600);
  
  Serial.println("RESETTIMER");

  Bluetooth.begin(9600);
  Serial.begin(9600);
  dht.begin();
  sensors.begin();
  
  currentBPMIndex = 0;
  for(int i = 0; i < (sizeof(lastNBPM)/sizeof(lastNBPM[0])); ++i)
  {
    lastNBPM[0] = 0;
  }
  
  // Initializes Timer2 to throw an interrupt every 2mS.
  TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
  TCCR2B = 0x06;     // DON’T FORCE COMPARE, 256 PRESCALER
  OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
  TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A
  sei();             // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED     
}

boolean hasStableBPM()
{
  int meanValue = 0;
  for(int i = 0; i < numberOfSmoothSamples; ++i)
  {
    meanValue += lastNBPM[i];
  }
  meanValue /= numberOfSmoothSamples;

  for(int i = 0; i < numberOfSmoothSamples; ++i)
  {
    if(lastNBPM[i] > maxBPM)
    {
      return false;
    }
    
    int val = lastNBPM[i] - meanValue;
    if(val < 0) val = -val;
    
    if(val > BPMtreshold)
    {
      return false;
    }
  }
  meanBPMValue = meanValue;
  return true;
}

int wasVisible = 0;

void loop()
{

  if (QS == true) // Quantified Self “QS” true when arduino finds a heartbeat
  {
    lastNBPM[currentBPMIndex] = BPM;
    currentBPMIndex = (currentBPMIndex + 1) % (sizeof(lastNBPM)/sizeof(lastNBPM[0]));

    bool validData = hasStableBPM();
    if(validData)
    {
      sensors.requestTemperatures();
      hum = dht.readHumidity();
      temp= dht.readTemperature();
      if (isnan(hum) || isnan(temp)) {
        return;
      }
      if (QS == true) {
        
        Serial.print(sensors.getTempCByIndex(0)); Serial.print(" "); Serial.print(temp); Serial.print(" "); Serial.print(hum); Serial.print(" "); Serial.println(meanBPMValue);
        
        Bluetooth.print(sensors.getTempCByIndex(0)); Bluetooth.print(" "); Bluetooth.print(temp); Bluetooth.print(" "); Bluetooth.print(hum); Bluetooth.print(" "); Bluetooth.println(meanBPMValue);
      }
        QS = false;

      int val = meanBPMValue;
      int numDigits = 0;
      while(val) {
        ++numDigits;
        val /= 10;
      }
      int spacesLeft = 4 - numDigits;
      
    }
    
    QS = false; // reset the Quantified Self flag for next time   
    delay(timeBetweenReads);
  }
}

ISR(TIMER2_COMPA_vect) //triggered when Timer2 counts to 124
{ 
  cli();                                      // disable interrupts while we do this
  
  Signal = analogRead(pulsePin);              // read the Pulse Sensor
  sampleCounter += 2;                         // keep track of the time in mS with this variable
  
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise
  
  //  find the peak and trough of the pulse wave
  if(Signal < thresh && N > (IBI/5)*3) // avoid dichrotic noise by waiting 3/5 of last IBI
  {     
      if (Signal < T) // T is the trough
      {                       
        T = Signal; // keep track of lowest point in pulse wave
      }
  }

  // keep track of highest point in pulse wave
  if(Signal > thresh && Signal > P)
  {          
    // thresh condition helps avoid noise
    P = Signal; // P is the peak
  }
  
  // signal surges up in value every time there is a pulse
  if (N > 250)
  {
    // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) )
    {       
      Pulse = true;                               // set the Pulse flag when we think there is a pulse
      IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
      lastBeatTime = sampleCounter;               // keep track of time for next pulse
        
      if(secondBeat)// if this is the second beat, if secondBeat == TRUE
      {
        secondBeat = false;// clear secondBeat flag
        for(int i=0; i<= sizeof(rate)/sizeof(rate[0]); i++) // seed the running total to get a realisitic BPM at startup
        {            
          rate[i] = IBI;                     
        }
      }

      if(firstBeat) // if it’s the first time we found a beat, if firstBeat == TRUE
      {                        
        firstBeat = false;                   // clear firstBeat flag
        secondBeat = true;                   // set the second beat flag
        sei();                               // enable interrupts again
        return;                              // IBI value is unreliable so discard it
      }  

      // keep a running total of the last 10 IBI values
      word runningTotal = 0; // clear the runningTotal variable   
      for(int i=0; i < sizeof(rate)/sizeof(rate[0]); i++)// shift data in the rate array
      {                
          rate[i] = rate[i+1];                  // and drop the oldest IBI value
          runningTotal += rate[i];              // add up the 9 oldest IBI values
      }
      rate[9] = IBI;                                          // add the latest IBI to the rate array
      runningTotal += rate[sizeof(rate)/sizeof(rate[0]) - 1]; // add the latest IBI to runningTotal

      runningTotal /= 10;                     // average the last 10 IBI values
      BPM = 60000/runningTotal;               // how many beats can fit into a minute? that’s BPM!
      QS = true;                              // set Quantified Self flag

    }                      
  }

  if (Signal < thresh && Pulse == true) // when the values are going down, the beat is over
  {   
      Pulse = false;                         // reset the Pulse flag so we can do it again
      amp = P - T;                           // get amplitude of the pulse wave
      thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
      P = thresh;                            // reset these for next time
      T = thresh;
  }

  if (N > 2500) // if 2.5 seconds go by without a beat
  {                           
      thresh = 512;                          // set thresh default
      P = 512;                               // set P default
      T = 512;                               // set T default
      lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date       
      firstBeat = true;                      // set these to avoid noise
      secondBeat = false;                    // when we get the heartbeat back
  }

  sei(); // enable interrupts when youre done!
}
