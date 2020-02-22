#include <DPM_8605.h>
#include "wiring_private.h"
#define Current false
#define Voltage true

DPM_8605 converter;
Uart dpmSerial (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// constants in I^2 = (X/t) + Y
float X = 0;
float Y = 0;

// Wire Resistance
float R = 0;
float activationTime = 1;
bool resistanceMeasured = false;

// Threshold resistance percentage change
float Rth = 0.09;
const float Rmin = 1.5;
const float Rmax = 50;

// A button which initiates everything
bool beginMeasurements = false;

// Variable Indicating whether test were completed
bool testsCompleted = false;

// Error Display
bool errorDisp = false;

// Error State
int8_t errorNum = 0;
// 101 - data not recieved when sending.
// 102 - variables not set when sending
// 103 - maximum current reached for tests.
// 104 - turn on reponse is not returned.
// 201 - activation current too high.
// 202 - activation current too low or negative, not going to work
// 203 - resistance is too low.
// 204 - resistance too high.

void setup() {
  Serial.begin(9600); // Serial that is used for communicating with computer
  delay(3000);
  delay(10);
  pinPeripheral(5, PIO_SERCOM_ALT); // Setting up pin 5 for RX
  pinPeripheral(6, PIO_SERCOM_ALT); // Setting up pin 6 for TX
  dpmSerial.begin(9600); // Serial that is passed for communication with DPM8605
  delay(10);

  converter.begin(dpmSerial, 3); // Starting communication with DPM8605
  Serial.println("Connected");
  converter.write(Voltage, 2, errorNum);
  //dpmSerial.println(":01w10=1000,");
  
  // Ready to start measuring
  Serial.println(F("Setup is finished. Send S to begin measurements"));
}

void loop() {

  if (Serial.available() > 0 && !beginMeasurements) {
    char cmd = Serial.read();
    if (cmd == 'S') {
      beginMeasurements = true;
      Serial.println(F("Start command was received. Starting measurements"));
    }
  }

  // Main Loop State Machine
  if (beginMeasurements && errorNum == 0) {
    if (!resistanceMeasured) {
      measureResistance();
    } else if (!testsCompleted) {
      runTests();
    } else {
      readyForActivation();
    }
  } else if (errorNum != 0) {
    if (!errorDisp) {
      // Display error here only once
      Serial.println("Error #" + String(errorNum));

      if (errorNum == 201 || errorNum == 202) {
        Serial.println(F("Try again in 5 seconds"));
      }

      errorDisp = true;
    }
    
    if (errorNum == 201 || errorNum == 202) {
      errorNum = 0;
      delay(5000);
      Serial.println(F("Ready for activation. Press right button to activate"));
    }
  }

}

void readyForActivation() {

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'A') {
      float t = activationTime;
      float c = sqrt((X / t) + Y);
      if ((c <= 5) && (c >= 0.2)) {
        // Display activating message
        Serial.println(F("Activation started..."));

        activate(c, t);

        // Display finished & ready to activate again
        Serial.println(F("Activation finished. Press right button to try again"));

      } else {
        if (c > 5) {
          errorNum = 201; // Current too high
        } else {
          errorNum = 202; // Current too low
        }
      }
    } else {
      int num = (int)cmd - 48;
      if (num > 0 && num < 10) {
        activationTime = num;
        Serial.println("Activation time changed to " + String(num));
      } else {
        Serial.println("Wrong command sent!");
      }
    }
    
  }
}

bool activate(float c, float &t) {
  converter.write(Current, c, errorNum);
  if (errorNum != 0) {
    return false;
  }
  
  // Record Time Stamp
  unsigned long timeStamp = millis();

  // Turn on the power
  converter.power(true, errorNum);
  
  if (errorNum != 0) {
    return false;
  }
  
  delay(150);
  // Measure voltage
  float v = converter.read(Voltage, errorNum);

  if (errorNum != 0) {
    converter.power(false, errorNum);
    return false;
  }

  // Calculate Initial Resistance
  //float rinit = (v / c);
  float rinit = R;

  // Set changing Resistance to Initial
  float r = rinit;
 
  // Run a while loop which will only be exited if time exceeds t+1 seconds or resistance drops low enough
  while ((r > rinit * (1 - Rth)) && (millis() - timeStamp < ((t + 1) * 1000))) {
    v = converter.read(Voltage, errorNum);
    // NEED TO IMPLEMENT ANOMALY  AND SLOPE DETECTION
    if (errorNum != 0) {
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
  // Turn off the power
  converter.power(false, errorNum);
  
  if (errorNum != 0) {
    return false;
  }

  if (r <= rinit * (1 - Rth)) {
    t = (millis()-timeStamp) / 1000.0;
    return true;
  } else {
    return false;
  }
}

void runTests() {
  float setV = R * 5; // Multiplying by 5 to get max voltage over what current could be, ensuring that it is CC.
  if (setV > 60) {
    setV = 60;
  } 
  Serial.println("Voltage is set to " + String(setV));
  converter.write(Voltage, setV, errorNum);
  delay(1000);

  if (errorNum != 0) {return ;}

  float c = 0.25; // Starting current for tests
  float deltaT[2];
  float cVal[2];
  uint8_t testNum = 1;
  int successfulTestCount = 0;

  while (successfulTestCount < 2) {
    // Display test num and successful test collected
    Serial.println("Starting Test #" + String(testNum) + ",Successes=" + String(successfulTestCount));

    float t = 12;
    bool success = activate(c, t);

    if (success) {
      cVal[successfulTestCount] = c;
      deltaT[successfulTestCount] = t;
      Serial.println("Success! c=" + String(c) + ", t=" + String(t));
      successfulTestCount++;
    } else if (errorNum != 0) {
      break;
    }
    c += 0.25;
    testNum++;
    if (c >= 5) {
      errorNum = 103;
      break;
    }

    Serial.println("Finished Test #" + String(testNum-1) + ",Successes=" + String(successfulTestCount));

    delay(40000); // 40 sec to cool down
  }

  if (errorNum != 0) {
    return;
  }
  // Calculating constants
  X = (sq(cVal[0]) - sq(cVal[1])) / ((1 / deltaT[0]) - (1 / deltaT[1]));
  Y = sq(cVal[0]) - (X / deltaT[0]);

  Serial.println("X=" + String(X) + ",Y=" + String(Y));

  testsCompleted = true;
  // Display that ready for activation
  Serial.println(F("Ready for activation. Press right button to activate"));
}

void measureResistance() {
  converter.write(Voltage, 5, errorNum);
  if (errorNum != 0) {
    R = 0;
    return;
  }

  converter.write(Current, 0.15, errorNum);
  
  if (errorNum != 0) {
    R = 0;
    return;
  }
  float Rarray[7];
  float Rdevi[7];
  float Rtot = 0;
  for (int i = 0; i < 6; i++) {
    // Display round i of resistance measurement
    Serial.println("Measuring Test " + String(i+1));
    converter.power(true, errorNum);
    delay(200);
    float c = converter.read(Current, errorNum);
    float v = converter.read(Voltage, errorNum);

    converter.power(false, errorNum);
    if (errorNum != 0) {
      break;
    }
    Rarray[i] = (v / c);
    Rtot = Rtot + (v / c);
    Serial.println("C=" + String(c) + ",V="+ String(v) + ",R=" + String(Rarray[i]));
    delay(5000); // 25s for 5 measurements
  }

  if (errorNum != 0) {
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
    if (Rdevi[i] < 2*Rdeviavg) {
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
    errorNum = 203;
    R = 0;
    return;
  } else if (R > Rmax) {
    errorNum = 204;
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
