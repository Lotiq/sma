#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define leftButtonPin 8
#define rightButtonPin 12
#define timeSettingKnob A0
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// constants in I^2 = (X/t) + Y
float X = 0;
float Y = 0;

// Wire Resistance
float R = 0;
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
int errorNum = 0;
// 101 - data not recieved when sending.
// 102 - variables not set when sending
// 103 - maximum current reached for tests.
// 104 - turn on reponse is not returned.
// 201 - activation current too high.
// 202 - activation current too low or negative, not going to work
// 203 - resistance is too low.
// 204 - resistance too high.

void setup() {
  Serial.begin(9600); // Change that number if communication rate is different

  pinMode(leftButtonPin, INPUT);
  pinMode(rightButtonPin, INPUT);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Initialize display. NEEDS AT LEAST 1024 Bytes in RAM to do that. If lower, will fail
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.display();
  
  // DISPLAY ready to start measuring
  displayHeaderMessage(F("BEGIN"));
  displayMainMessage(F("Press LEFT button to begin measurements"));
}

void loop() {
  if (digitalRead(leftButtonPin) == HIGH && beginMeasurements == false) {
    beginMeasurements = true;
    displayHeaderMessage(F("RESISTANCE"));
    displayMainMessage(F("Starting measuruments..."));
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
      display.clearDisplay();
      displayHeaderMessage("ERROR #" + String(errorNum));
      if (errorNum == 201 || errorNum == 202) {
        displayMainMessage(F("Try Again once this message dissapears."));
      }
      errorDisp = true;
    }
    
    if (errorNum == 201 || errorNum == 202) {
      errorNum = 0;
      delay(6000);
      displayHeaderMessage(F("READY"));
      displayMainMessage(F("Ready for activation. Press right button to activate"));
    }
  }

}

void readyForActivation() {

  if (digitalRead(rightButtonPin) == HIGH) {
    float t = map(analogRead(timeSettingKnob), 1023, 0, 1, 9);
    float c = sqrt((X / t) + Y);
    if ((c <= 5) && (c >= 0.2)) {
      // Display activating message
      displayMainMessage(F("Activating..."));

      activate(c, t);

      // Display finished & ready to activate again
      displayMainMessage(F("Finished Activating.Press RIGHT button to activate again."));

    } else {
      if (c > 5) {
        errorNum = 201; // Current too high
      } else {
        errorNum = 202; // Current too low
      }
    }
  }
}

bool activate(float c, float &t) {
  setVariable('c', c);
  if (errorNum != 0) {
    return false;
  }
  
  // Record Time Stamp
  unsigned long timeStamp = millis();

  // Turn on the power
  turnOn(true);
  if (errorNum != 0) {
    return false;
  }
  
  delay(150);
  // Measure voltage
  float v = readVariable('v', errorNum);

  if (errorNum != 0) {
    turnOn(false);
    return false;
  }

  // Calculate Initial Resistance
  //float rinit = (v / c);
  float rinit = R;

  // Set changing Resistance to Initial
  float r = rinit;
 
  // Run a while loop which will only be exited if time exceeds t+1 seconds or resistance drops low enough
  while ((r > rinit * (1 - Rth)) && (millis() - timeStamp < ((t + 1) * 1000))) {
    v = readVariable('v', errorNum);
    // NEED TO IMPLEMENT ANOMALY  AND SLOPE DETECTION
    if (errorNum != 0) {
      break;
    }
    r = (v / c);
    if (r > rinit) {
      rinit = r; // ensure that rinit gets the max value, and dropping is counted from that value.
    }

    displaySubmainMessage(String(rinit) + "," + String(r));
    // Set delay so the while loop doesn't run too fast
    delay(60);
  }
  // Turn off the power
  turnOn(false);

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
  int successfulTestCount = 0;
  float setV = R * 3;
  if (setV > 60) {
    displayMainMessage(F("Maximum Voltage. Be aware!"));
    setVariable('v', 60);
    delay(5000);
  } else if (setV > 12) {
    displayMainMessage(F("High Voltage. Be aware!"));
    setVariable('v', setV);
    delay(5000);
  } else {
    setVariable('v', setV);
  }
  if (errorNum != 0) {
    return ;
  }
  float c = 0.25; // Starting current for tests
  float deltaT[2];
  float cVal[2];
  uint8_t testNum = 1;

  while (successfulTestCount < 2) {
    // Display test num and successful test collected
    displayMainMessage("Test #" + String(testNum) + ",Successes=" + String(successfulTestCount));

    float t = 12;
    bool success = activate(c, t);

    if (success) {
      cVal[successfulTestCount] = c;
      deltaT[successfulTestCount] = t;
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
    delay(40000); // 40 sec to cool down
  }

  if (errorNum != 0) {
    return;
  }
  // Calculating constants
  //displayMainMessage("cVal1=" + String(cVal[0]) + ",cVal2=" + String(cVal[1]) + ",dt1=" + String(deltaT[0])+ ",dt2="+ String(deltaT[1]));
  X = (sq(cVal[0]) - sq(cVal[1])) / ((1 / deltaT[0]) - (1 / deltaT[1]));
  Y = sq(cVal[0]) - (X / deltaT[0]);

  // Display values X and Y
  displayMainMessage("X=" + String(X) + ",Y=" + String(Y));
  delay(5000);


  testsCompleted = true;
  // Display that ready for activation
  display.clearDisplay();
  displayHeaderMessage(F("READY"));
  displayMainMessage(F("Ready for activation. Press right button to activate"));

}

void measureResistance() {
  setVariable('v', 5);
  
  if (errorNum != 0) {
    R = 0;
    return;
  }
  
  setVariable('c', 0.15);
  
  if (errorNum != 0) {
    R = 0;
    return;
  }
  float Rarray[7];
  float Rdevi[7];
  float Rtot = 0;
  for (int i = 0; i < 6; i++) {
    // Display round i of resistance measurement
    displayMainMessage("Measuring Test " + String(i+1));
    turnOn(true);
    delay(200);
    float c = readVariable('c', errorNum);
    float v = readVariable('v', errorNum);
    turnOn(false);
    if (errorNum != 0) {
      break;
    }
    displaySubmainMessage("C=" + String(c) + ",V="+ String(v));
    Rarray[i] = (v / c);
    Rtot = Rtot + (v / c);
    delay(5000); // 15s for 5 measurements
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
  }
  float Rdeviavg = 0;
  for (int i = 0; i < 6; i++) {
    Rdeviavg = Rdeviavg + Rdevi[i];
  }

  Rdeviavg = Rdeviavg / 6;
  int k = 0;
  for (int i = 0; i < 6; i++) {
    if (Rdevi[i] < 2*Rdeviavg) {
      k += 1;
      R = R + Rarray[i];
    }
  }
  R = R / k; // Only counting the non-outliers
  displayMainMessage("k = " + String(k));
  delay(3000);
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
  // Display resistance
  displayMainMessage("R = " + String(R));
  delay(5000);

  // Display that ready for tests
  display.clearDisplay();
  displayHeaderMessage(F("TESTS"));
  displayMainMessage(F("Starting tests..."));
}

void serialListen(String &cmd, bool &stringComplete) {
  unsigned long errorTimer = millis();
  while ( (!stringComplete) && (millis() - errorTimer) < 200 ) {
    if (Serial.available()) {
      char inChar = (char)Serial.read();
      if (inChar == '\n') {
        stringComplete = true;
      } else {
        cmd += inChar;
      }
    }
  }
}

void turnOn(bool on) {

  int retries = 0;
  String cmd = "";
  bool stringComplete = false;
  
  do {
    // Send command
    if (on) {
      Serial.print(":01w12=1,\r\n");
    } else {
      Serial.print(":01w12=0,\r\n");
    }

    // Listen for reply
    serialListen(cmd, stringComplete);

    // Add a retry
    retries += 1;
    
  } while ((!stringComplete) && (retries < 2));

  if (!stringComplete) {
    errorNum = 104; // no response from setting variable
    return ;
  }
  
  delay(1);
}

void setVariable(char vc, float value) {
  int x = 0;
  String function = "";

  if (vc == 'v' || vc == 'V') {
    x = floor(value * 100);
    function = "10";
  } else {
    x = floor(value * 1000);
    function = "11";
  }

  int retries = 0;
  String cmd = "";
  bool stringComplete = false;
  
  do {
    // Send command
    Serial.print(":01w" + function + "=" + String(x) + ",\r\n");

    // Listen for reply
    serialListen(cmd, stringComplete);

    // Add a retry
    retries += 1;
  } while ((!stringComplete) && (retries < 2));

  if (!stringComplete) {
    errorNum = 102; // no response from setting variable
    return ;
  }
}

float readVariable(char vc, int &errorNum) {
  bool stringComplete = false;
  int retries = 0;
  String cmd = "";

  do {
    // Send command
    if (vc == 'v' || vc == 'V') {
      Serial.print(F(":01r30=0,\r\n"));
    } else {
      Serial.print(F(":01r31=0,\r\n"));
    }

    // Listen for a reply
    serialListen(cmd, stringComplete);

    // Add a retry
    retries += 1;

  } while ((!stringComplete) && (retries < 2)); // Stop running either when data received or 2 attempts happened.

  if (!stringComplete) {
    errorNum = 101; // data not recieved
    return 0;
  }

  float x = processString(cmd);

  if (vc == 'v' || vc == 'V') {
    return (x / 100);
  } else {
    return (x / 1000);
  }
}

float processString(String str) {
  // remove first 7 unnecessary characters (e.g. ":01r11=3500.") and full stop at the end
  str.remove(0, 7);
  str.replace(".", "");
  return atof(str.c_str());
}

void displayHeaderMessage(String message) {
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.fillRect(0, 0, 128, 16, BLACK);
  display.println(message);
  display.display();
  delay(5);
}

void displayMainMessage(String message) {
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.fillRect(0, 16, 128, 48, BLACK);
  display.println(message);
 
  display.display();
  delay(5);
}

void displaySubmainMessage(String message) {
  display.setTextSize(1);
  display.setCursor(0, 48);
  display.fillRect(0, 48, 128, 64, BLACK);
  display.println(message);
 
  display.display();
  delay(5);
}
