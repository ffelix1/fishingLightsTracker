#include <Simpletools.h>
#include <Servo.h>
Servo servo1;

int pinFeedback1 = 11;     // For pulse_in, print, scan etc...
int pinControl1 = 9;      // For servo pulse control

volatile int angle, targetAngle;              // Global shared variables
volatile int Kp = 1;                          // Proportional constant   

void feedback360();                           // Position monitoring
void control360();                            // Position control

   
    
void feedback360()
{                            // Cog keeps angle variable updated
  int unitsFC = 360;                          // Units in a full circle
  int dutyScale = 1000;                       // Scale duty cycle to 1/1000ths
  int dcMin = 29;                             // Minimum duty cycle
  int dcMax = 971;                            // Maximum duty cycle
  int q2min = unitsFC/4;                      // For checking if in 1st quadrant
  int q3max = q2min * 3;                      // For checking if in 4th quadrant
  int turns = 0;                              // For tracking turns
  // dc is duty cycle, theta is 0 to 359 angle, thetaP is theta from previous
  // loop repetition, tHigh and tLow are the high and low signal times for 
  // duty cycle calculations.
  int dc, theta, thetaP, tHigh, tLow;         

  // Measure feedback signal high/low times.
  tLow = pulseIn(pinFeedback1, 0);            // Measure low time 
  tHigh = pulseIn(pinFeedback1, 1);           // Measure high time

  // Calcualte initial duty cycle and angle.
  dc = (dutyScale * tHigh) / (tHigh + tLow);
  theta = (unitsFC - 1) - ((dc - dcMin) * unitsFC) / (dcMax - dcMin + 1);
  thetaP = theta;

  while(1)                                    // Main loop for this cog
  {
    // Measure high and low times, making sure to only take valid cycle
    // times (a high and a low on opposite sides of the 0/359 boundary
    // will not be valid.
    int tCycle = 0;                           // Clear cycle time
    while(1)                                  // Keep checking
    {
      tHigh = pulseIn(pinFeedback1, 1);       // Measure time high
      tLow = pulseIn(pinFeedback1, 0);        // Measure time low
      tCycle = tHigh + tLow;
      if((tCycle > 1000) && (tCycle < 1200))  // If cycle time valid 
        break;                                // break from loop
    }      
    dc = (dutyScale * tHigh) / tCycle;        // Calculate duty cycle
    
    // This gives a theta increasing int the
    // counterclockwise direction.
    theta = (unitsFC - 1) -                   // Calculate angle
            ((dc - dcMin) * unitsFC) 
            / (dcMax - dcMin + 1);

    if(theta < 0)                             // Keep theta valid
      theta = 0; 
    else if(theta > (unitsFC - 1)) 
      theta = unitsFC - 1;

    // If transition from quadrant 4 to  
    // quadrant 1, increase turns count. 
    if((theta < q2min) && (thetaP > q3max))
      turns++;
    // If transition from quadrant 1 to  
    // quadrant 4, decrease turns count. 
    else if((thetaP < q2min) && (theta > q3max))
      turns --;

    // Construct the angle measurement from the turns count and
    // current theta value.
    if(turns >= 0)
      angle = (turns * unitsFC) + theta;
    else if(turns <  0)
      angle = ((turns + 1) * unitsFC) - (unitsFC - theta);

    thetaP = theta;                           // Theta previous for next rep
  }
}

void control360()                             // Cog for control system
{
  servo1.write(0);                            // Start servo control cog
  
  int errorAngle, output, offset;             // Control system variables
  
  while(1)                                    // Main loop for this cog
  {
    errorAngle = targetAngle - angle;         // Calculate error
    output = errorAngle * Kp;                 // Calculate proportional 
    if(output > 200) output = 200;            // Clamp output
    if(output < -200) output = -200;
    if(errorAngle > 0)                        // Add offset
      offset = 30;
    else if(errorAngle < 0)
      offset = -30;
    else
      offset = 0;     
    servo1.write(output + offset);            // Set output
    delay(20);                                // Repeat after 20 ms
  }    
}  


void setup() {
  Serial.begin(9600);
  
  servo1.attach(pinControl1);
  
  delay(100);
}

void loop() { 
  
  Serial.print("Enter angle: ");                   // Prompt user for angle
    targetAngle = Serial.read();                      // Get entered angle
    Serial.print("\n");                            // Next line
             feedback360();
              control360();    
    while(abs(targetAngle - angle) > 4)       // Display until close to finish
    {
         // Display target & measured

      Serial.print(targetAngle);
      Serial.println(angle);
      
      delay(50);                              // ...every 50 ms
    } 
}
