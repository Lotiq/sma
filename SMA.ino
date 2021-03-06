#include <DPM_8600.h>
#include "wiring_private.h"

DPM_8600 converter(1);
Uart dpmSerial (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// constants in I^2 = (X/t) + Y
float X = 0;
float Y = 0;

// Wire Resistance
float R = 0;
float activationTime = 3;
bool resistanceMeasured = false;

// Threshold resistance percentage change
float Rth = 0.06;
const float Rmin = 1.5;
const float Rmax = 50;

// Boundary values for time and current
float cMin;
float cMax;
float tMin;
float tMax;

// A button which initiates everything
bool beginMeasurements = false;

// Variable Indicating whether test were completed
bool testsCompleted = false;

// Error Display
bool errorDisp = false;

// Error State
int8_t errorNum = 1;

  // 1 - NO ERROR
  // -1 - error on begining DPM8600.
  // -10 - error on voltage reading
  // -11 - error on current reading.
  // -12 - error on power reading.
  // -13 - error on CC/CV reading.
  // -14 - error on max current reading.
  // -15 - error on temperature reading.
  // -20 - wrong value sent to power writing function
  // -21 - error setting current
  // -22 - error setting power on/off.
  // -23 - error setting voltage
  // -24 - error setting current and voltage
  // -30 - too much current during tests
  // -31 - too little resistance
  // -32 - too high resistance
  // -33 - no connection or too high resistance

void setup() {
  Serial.begin(9600); // Serial that is used for communicating with computer
  while (!Serial) {;} // Wait for serial screen on the computer to be open
  pinPeripheral(5, PIO_SERCOM_ALT); // Setting up pin 5 for RX
  pinPeripheral(6, PIO_SERCOM_ALT); // Setting up pin 6 for TX
  dpmSerial.begin(19200); // Serial that is passed for communication with DPM8605
  errorNum = converter.begin(dpmSerial, 3); // Starting communication with DPM8605 with 3 retries max.
  
  // Ready to start measuring
  Serial.println(F("Setup is finished. Send S to begin measurements"));
}

void loop() {

  if (Serial.available() > 0 && !beginMeasurements && errorNum == 1) {
    char cmd = Serial.read();
    if (cmd == 'S') {
      beginMeasurements = true;
      Serial.println(F("Start command was received. Starting measurements"));
    }
  }

  // Main Loop State Machine
  if (beginMeasurements && errorNum == 1) {
    if (!resistanceMeasured) {
      measureResistance();
    } else if (!testsCompleted) {
      runTests();
      displayLimits();
    } else {
      readyForActivation();
    }
  } else if (errorNum != 1) {
    if (!errorDisp) {
      Serial.println("Error #" + String(errorNum));
      errorDisp = true;
    }
  }
}

float currentFor(float t){
  return sqrt((X / t) + Y);
}

float timeFor(float c){
  return (X/(sq(c)-Y));
}

void displayLimits() {
  cMin = currentFor(10);
  cMax = currentFor(1);
  tMin = 1;
  tMax = 10;
  if (cMin < 0.1) {
    tMax = timeFor(0.1);
    cMin = currentFor(tMax);
  }

  if (cMax > 5) {
    tMin = timeFor(5);
    cMax = currentFor(tMin);
  }
  activationTime = timeFor(cMax - (cMax-cMin)*0.1);
  Serial.print(F("Limits based on the equation are: fastest->"));
  Serial.print(cMax);
  Serial.print(F(" for "));
  Serial.print(tMin);
  Serial.print(F(", slowest->"));
  Serial.print(cMin);
  Serial.print(F(" for "));
  Serial.println(tMax);
}

void readyForActivation() {

  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "A") {

      float t = activationTime;
      float c = currentFor(activationTime);
      
      // Display activating message
      Serial.println(F("Activation started..."));

      activate(c, t);

      // Display finished & ready to activate again
      Serial.println(F("Activation finished"));

    } else {
      // Read the time
      float t = cmd.toFloat();

      // Check if command is ok or time is within limit range
      if (t < tMin || t > tMax) {
        Serial.println(F("Outside of range time or wrong command. Try again within limits"));
      } else {
        activationTime = t;
        float c = currentFor(activationTime);
        // Display new settings;
        Serial.println("Activation time set to " + String(activationTime) + ", current necessary=" + String(c));
      }
    }
    
  }
}

bool activate(float c, float &t) {
  Serial.println("Current is set to " + String(c));
  errorNum = converter.write('c', c);
  float setV = R * c * 1.3; // Multiplying by 5 to get max voltage over what current could be, ensuring that it is CC.
  Serial.println("Voltage is set to " + String(setV));
  errorNum = converter.write('v', setV);

  if (errorNum != 1) {
    return false;
  }
  
  // Record Time Stamp
  unsigned long timeStamp = millis();

  // Turn on the power
  converter.power(true);
  
  delay(500);
  
  // Measure voltage
  float v = converter.read('v');

  if (v < 0) {
    errorNum = floor(v);
    converter.power(false);
    return false;
  }

  // Calculate Initial Resistance
  float rinit = R;

  // Set changing Resistance to Initial
  float r = rinit;
 
  // Run a while loop which will only be exited if time exceeds t+1 seconds or resistance drops low enough
  while ((r > rinit * (1 - Rth)) && (millis() - timeStamp < ((t + 1) * 1000))) {
    v = converter.read('v');
    // NEEDS SLOPE DETECTION
    if (v < 0) {
      errorNum = floor(v);
      break;
    }
    r = (v / c);
    if (r > rinit) {
      rinit = r; // ensure that rinit gets the max value, and dropping is counted from that value.
    }

    Serial.println("Rinitial=" + String(rinit) + ", Rcurrent=" + String(r));
    // Set delay so the while loop doesn't run too fast
    delay(60);
  }

  Serial.println("CONCLUDING: Rinitial=" + String(rinit) + ", Rcurrent=" + String(r));
  
  // Turn off the power
  converter.power(false);

  if (r <= rinit * (1 - Rth)) {
    t = (float)(millis()-timeStamp) / 1000.0;
    return true;
  } else {
    return false;
  }
}

void runTests() {
  float c = 0.2; // Starting current for tests
  float deltaT[2];
  float cVal[2];
  uint8_t testNum = 1;
  int successfulTestCount = 0;

  while (successfulTestCount < 2) {
    // Display test num and successful test collected
    Serial.println("Starting Test #" + String(testNum) + ",Successes=" + String(successfulTestCount));

    float t = 10;
    bool success = activate(c, t);

    if (success) {
      cVal[successfulTestCount] = c;
      deltaT[successfulTestCount] = t;
      Serial.println("Success! c=" + String(c) + ", t=" + String(t));
      successfulTestCount++;
    } else if (errorNum != 1) {
      break;
    }
    if (success) {
      c+= 0.05;
    } else {
      c += 0.1;
    }
    testNum++;
    if (c >= 5) {
      errorNum = -30;
      break;
    }

    Serial.println("Finished Test #" + String(testNum-1) + ",Successes=" + String(successfulTestCount));

    delay(40000); // 40 sec to cool down
  }

  if (errorNum != 1) {
    return;
  }
  // Calculating constants
  X = (sq(cVal[0]) - sq(cVal[1])) / ((1 / deltaT[0]) - (1 / deltaT[1]));
  Y = sq(cVal[0]) - (X / deltaT[0]);

  Serial.println("X=" + String(X) + ",Y=" + String(Y));

  testsCompleted = true;
  // Display that ready for activation
  Serial.println(F("Ready for activation. Send A to active or a number to set time"));
}

void measureResistance() {
  errorNum = converter.writeVC(5, 0.1);
  if (errorNum != 1) {
    R = 0;
    return;
  }
  
  float Rarray[7];
  float Rdevi[7];
  float Rtot = 0;
  for (int i = 0; i < 6; i++) {
    // Display round i of resistance measurement
    Serial.println("Measuring Test " + String(i+1));
    converter.power(true);
    delay(2000);
    float c = converter.read('c');
    float v = converter.read('v');

    converter.power(false);
    if (v < 0) {
      errorNum = floor(v);
      break;
    } else if (c < 0) {
      errorNum = floor(c);
      break;
    } else if (c < 0.01) {
      errorNum = -33;
      break;
    }
    Rarray[i] = (v / c);
    Rtot = Rtot + (v / c);
    Serial.print("C=");
    Serial.print(c, 3);
    Serial.println(",V="+ String(v) + ",R=" + String(Rarray[i]));
    //Serial.println("C=" + String(c) + ",V="+ String(v) + ",R=" + String(Rarray[i]));
    delay(5000); // 25s for 5 measurements
  }

  if (errorNum != 1) {
    R = 0;
    return;
  }
  
  float Ravg = Rtot / 6;
  for  (int i = 0; i < 6; i++) {
    float devi = Rarray[i] - Ravg;
    if (devi < 0) {devi = -devi;} // Modulus
    Rdevi[i] = devi;

    Serial.print("devi"+String(i+1) + "=" + String(devi) + ", ");
  }
  Serial.println("");
  float Rdeviavg = 0;
  for (int i = 0; i < 6; i++) {
    Rdeviavg = Rdeviavg + Rdevi[i];
  }
  Rdeviavg = Rdeviavg / 6;
  Serial.println("Average deviation=" + String(Rdeviavg));
  int k = 0;
  for (int i = 0; i < 6; i++) {
    if (Rdevi[i] <= 2*Rdeviavg) {
      k += 1;
      R = R + Rarray[i];
      Serial.print(String(i+1) + "=pass, ");
    } else {
      Serial.print(String(i+1) + "=fail, ");
    }
  }
  Serial.println("");
  R = R / k; // Only counting the non-outliers
  Serial.println("R="+String(R) + ",k=" + String(k));
  if (R < Rmin) {
    errorNum = -31;
    R = 0;
    return;
  } else if (R > Rmax) {
    errorNum = -32;
    R = 0;
    return;
  }

  resistanceMeasured = true;

  // Display that ready for tests
  Serial.println("Resistance measuments are finished, starting tests...");
}

void SERCOM0_Handler()
{
    dpmSerial.IrqHandler();
}
