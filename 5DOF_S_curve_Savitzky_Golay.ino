#include <Servo.h>
#include <Wire.h>
#include <U8g2lib.h>

// OLED Display setup
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Create array of servo objects
Servo myservos[5];

// Pin definitions
const int buttonPin = 2;
const int potPins[] = {A0, A1, A2, A3, A4}; // 5 potentiometer pins
const int sButton = 3;  // Store
const int tButton = 4;  // Terminal/Ready
const int yButton = 5;  // Yes (loop)
const int nButton = 6;  // No (single run)
const int qButton = 7;  // Quit
const int ledPin = 8;   // Mode indicator LED
const int pressureSensorPin = A5; // Pressure sensor pin

// Servo pins
const int servoPins[] = {9, 10, 11, 12, 13}; // 5 servo pins

// Program states
int buttonState = HIGH;
bool autoMode = false;
bool recording = false;
bool ready = false;
bool executeLoop = false;
bool stopExecution = false;
bool waitingForYN = false;  // New flag to indicate waiting for Y/N input
int selectedServo = 0;  // Currently selected servo in auto mode
String runMode = "";    // To display the current run mode (Loop/Single)

// Storage for positions
int storedPositions[5][10]; // [servo][position]
int currentPositions[5];    // Current positions of all servos
int previousPositions[5];   // Store previous positions
int targetPositions[5];     // Target positions for smooth movement
float currentVelocities[5]; // Current velocities for smooth movement
int posCount = 0;
int delayBetweenSteps = 2500; // Increased for smoother motion

// Motion profile parameters
const float ACCEL_TIME_RATIO = 0.25;  // Acceleration phase is 25% of total time
const float DECEL_TIME_RATIO = 0.25;  // Deceleration phase is 25% of total time
const float CRUISE_TIME_RATIO = 0.5;  // Cruise phase is 50% of total time

// Savitzky-Golay filter parameters
const int SG_WINDOW_SIZE = 9;  // Must be odd number (increased from 7)
const int SG_HALF_WINDOW = SG_WINDOW_SIZE / 2;
// Savitzky-Golay coefficients for 4th order polynomial (window size 9)
const float SG_COEFFS[] = {-21, 14, 39, 54, 59, 54, 39, 14, -21};
const float SG_NORM = 231.0;  // Normalization factor (sum of coefficients)

// Multiple filter stages
const int POT_SAMPLES = 10;    // Number of samples for potentiometer averaging
int potBuffer[5][POT_SAMPLES]; // Buffer for potentiometer readings
int potIndex[5] = {0, 0, 0, 0, 0};

// Filter buffers for Savitzky-Golay
int sgBuffer[5][SG_WINDOW_SIZE];  // Circular buffer for each servo
int sgBufferIndex[5] = {0, 0, 0, 0, 0};  // Current index in circular buffer

// Velocity control
float maxVelocity = 1.0;       // Maximum velocity for servos (degrees per update)
float acceleration = 0.05;     // Acceleration rate (degrees per update^2)
float servoVelocity[5] = {0, 0, 0, 0, 0}; // Current velocity of each servo

// Exponential smoothing parameters
const float ALPHA = 0.15;      // Smoothing factor (0.0 to 1.0) - smaller = smoother

// Pressure sensor variables
int pressureValue = 0;

// Timing variables
unsigned long lastDebounceTime = 0;
unsigned long lastMoveTime = 0;
unsigned long moveStartTime = 0;
const unsigned long debounceDelay = 500;
const int updateInterval = 20;  // Time between position updates (ms) - increased slightly for stability

// Dead zone for potentiometers
const int DEADZONE = 8;        // Dead zone size in servo degrees

// Servo configuration
const int SERVO_MIN_PULSE = 500;   // Minimum pulse width (microseconds)
const int SERVO_MAX_PULSE = 2500;  // Maximum pulse width (microseconds)

void setup() {
    Serial.begin(9600);
    
    // Initialize OLED
    u8g2.begin();
    
    // Initialize buttons
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(sButton, INPUT_PULLUP);
    pinMode(tButton, INPUT_PULLUP);
    pinMode(yButton, INPUT_PULLUP);
    pinMode(nButton, INPUT_PULLUP);
    pinMode(qButton, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
    
    // Initialize filter buffers
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < SG_WINDOW_SIZE; j++) {
            sgBuffer[i][j] = 0;
        }
        for (int j = 0; j < POT_SAMPLES; j++) {
            potBuffer[i][j] = 0;
        }
    }
    
    // Attach all servos with specific pulse ranges and initialize positions
    for(int i = 0; i < 5; i++) {
        myservos[i].attach(servoPins[i], SERVO_MIN_PULSE, SERVO_MAX_PULSE);
        previousPositions[i] = 0;
        currentPositions[i] = 0;
        targetPositions[i] = 0;
        currentVelocities[i] = 0;
        
        // Pre-fill the filter buffers with initial positions
        for (int j = 0; j < SG_WINDOW_SIZE; j++) {
            sgBuffer[i][j] = 0;
        }
        for (int j = 0; j < POT_SAMPLES; j++) {
            potBuffer[i][j] = 0;
        }
        
        // Move to home position
        myservos[i].writeMicroseconds(SERVO_MIN_PULSE);
        delay(100); // More time between servo initializations
    }
    
    Serial.println("Advanced Robotic Arm Control System Initialized");
    delay(500); // Additional startup delay for servo stabilization
}

// Apply Savitzky-Golay filter to smooth servo movements
int applySavitzkyGolayFilter(int servoIndex, int newValue) {
    // Add new value to filter buffer
    sgBuffer[servoIndex][sgBufferIndex[servoIndex]] = newValue;
    sgBufferIndex[servoIndex] = (sgBufferIndex[servoIndex] + 1) % SG_WINDOW_SIZE;
    
    // Apply Savitzky-Golay convolution
    float filteredValue = 0;
    
    for (int i = 0; i < SG_WINDOW_SIZE; i++) {
        int bufIdx = (sgBufferIndex[servoIndex] - 1 - i + SG_WINDOW_SIZE) % SG_WINDOW_SIZE;
        filteredValue += sgBuffer[servoIndex][bufIdx] * SG_COEFFS[i];
    }
    
    // Normalize and round the result
    filteredValue = round(filteredValue / SG_NORM);
    
    // Ensure the output is within valid servo range
    return constrain((int)filteredValue, 0, 180);
}

// Apply additional exponential smoothing
int applyExponentialSmoothing(int servoIndex, int newValue) {
    // Exponential moving average: currentValue = alpha*newValue + (1-alpha)*currentValue
    currentPositions[servoIndex] = ALPHA * newValue + (1 - ALPHA) * currentPositions[servoIndex];
    return round(currentPositions[servoIndex]);
}

// Apply averaging filter to potentiometer readings
int applyPotFilter(int servoIndex, int newValue) {
    // Add new value to pot buffer
    potBuffer[servoIndex][potIndex[servoIndex]] = newValue;
    potIndex[servoIndex] = (potIndex[servoIndex] + 1) % POT_SAMPLES;
    
    // Calculate average, ignoring highest and lowest values to reduce outlier effects
    int values[POT_SAMPLES];
    for (int i = 0; i < POT_SAMPLES; i++) {
        values[i] = potBuffer[servoIndex][i];
    }
    
    // Simple bubble sort to find outliers
    for (int i = 0; i < POT_SAMPLES-1; i++) {
        for (int j = 0; j < POT_SAMPLES-i-1; j++) {
            if (values[j] > values[j+1]) {
                int temp = values[j];
                values[j] = values[j+1];
                values[j+1] = temp;
            }
        }
    }
    
    // Calculate average excluding extreme values
    long sum = 0;
    for (int i = 1; i < POT_SAMPLES-1; i++) {  // Skip first and last (min and max)
        sum += values[i];
    }
    
    return sum / (POT_SAMPLES - 2);  // Average of middle values
}

// S-curve motion profile with improved smoothness
float sCurveProfile(float t, float totalTime) {
    // Ensure time is within bounds
    t = constrain(t, 0, totalTime);
    
    // Calculate phase times
    float accelTime = totalTime * ACCEL_TIME_RATIO;
    float cruiseTime = totalTime * CRUISE_TIME_RATIO;
    float decelTime = totalTime * DECEL_TIME_RATIO;
    float cruiseStartTime = accelTime;
    float decelStartTime = accelTime + cruiseTime;
    
    // Calculate position along S-curve
    if (t < accelTime) {
        // Acceleration phase - cubic curve for smoother start
        float normalizedT = t / accelTime;
        return 0.5 * normalizedT * normalizedT * normalizedT;
    } else if (t < decelStartTime) {
        // Cruise phase - linear
        return 0.5 + (t - cruiseStartTime) / totalTime / CRUISE_TIME_RATIO * CRUISE_TIME_RATIO;
    } else {
        // Deceleration phase - cubic curve for smoother end
        float decelProgress = (t - decelStartTime) / decelTime;
        float normalizedT = 1.0 - decelProgress;
        return 1.0 - 0.5 * normalizedT * normalizedT * normalizedT;
    }
}

// Convert degrees to microseconds for more precise servo control
int degreesToMicroseconds(int degrees) {
    // Map 0-180 degrees to SERVO_MIN_PULSE - SERVO_MAX_PULSE microseconds
    return map(degrees, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

void moveServoSmooth(int servoIndex, int targetPos, unsigned long elapsedTime, unsigned long totalTime) {
    // Ensure target position is within valid range
    targetPos = constrain(targetPos, 0, 180);
    
    // Store target position
    targetPositions[servoIndex] = targetPos;
    
    // Calculate current position based on S-curve profile
    int startPos = previousPositions[servoIndex];
    float progress = sCurveProfile(elapsedTime, totalTime);
    int newPos = startPos + (targetPos - startPos) * progress;
    
    // Apply double filtering for maximum smoothness
    newPos = applySavitzkyGolayFilter(servoIndex, newPos);
    newPos = applyExponentialSmoothing(servoIndex, newPos);
    
    // Only update if position has changed by a meaningful amount
    if (abs(newPos - currentPositions[servoIndex]) >= 1) {
        // Use writeMicroseconds for more precise control
        int pulseWidth = degreesToMicroseconds(newPos);
        myservos[servoIndex].writeMicroseconds(pulseWidth);
        currentPositions[servoIndex] = newPos;
    }
}

// Dead zone function to prevent tiny movements
int applyDeadZone(int value, int center, int threshold) {
    if (abs(value - center) < threshold) {
        return center;
    }
    return value;
}

void updateDisplay() {
    u8g2.clearBuffer();
    
    // Title with smaller font
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.setCursor((128 - u8g2.getStrWidth("ROBOTIC ARM CONTROL")) / 2, 7);
    u8g2.print("ROBOTIC ARM CONTROL");
    
    // Draw top separator line
    u8g2.drawHLine(0, 9, 128);
    
    // Left side - Mode and Status
    u8g2.setCursor(2, 20);
    u8g2.print("Mode:");
    
    // Display mode text
    String modeText = autoMode ? "AUTO" : "MANUAL";
    u8g2.setCursor(36, 20);
    u8g2.print(modeText);
    
    if (autoMode) {
        if (recording) {
            u8g2.setCursor(2, 30);
            u8g2.print("Rec Pos:");
            
            // Display position count
            u8g2.setCursor(46, 30);
            u8g2.print(posCount);
            
            u8g2.setCursor(2, 40);
            u8g2.print("Sel:S");
            
            // Display selected servo
            u8g2.setCursor(36, 40);
            u8g2.print(selectedServo + 1);
        } else if (waitingForYN) {
            // Show Y/N prompt
            u8g2.setCursor(2, 30);
            u8g2.print("Loop? Y/N");
        } else if (ready) {
            // Show running mode
            u8g2.setCursor(2, 30);
            u8g2.print("Running:");
            u8g2.setCursor(2, 40);
            u8g2.print(runMode);
        }
    }
    
    // Draw vertical separator
    u8g2.drawVLine(63, 10, 44);
    
    // Right side - Servo Positions
    for(int i = 0; i < 5; i++) {
        u8g2.setCursor(66, 20 + (i * 9));
        u8g2.print("S");
        u8g2.print(i + 1);
        u8g2.print(":");
        
        // Display servo values
        u8g2.setCursor(120 - u8g2.getStrWidth(String(currentPositions[i]).c_str()), 20 + (i * 9));
        u8g2.print(currentPositions[i]);
    }
    
    // Bottom section - only draw line on left side
    u8g2.drawHLine(0, 54, 63);
    
    // Pressure value at bottom left
    u8g2.setCursor(2, 63);
    u8g2.print("Pressure:");
    
    // Display pressure value
    u8g2.setCursor(51, 63);
    u8g2.print(pressureValue);
    
    u8g2.sendBuffer();
}

// Use velocity-based movement for manual mode
int moveWithVelocityControl(int servoIndex, int targetPos) {
    int currentPos = currentPositions[servoIndex];
    float& velocity = servoVelocity[servoIndex];
    
    // Calculate distance to target
    int error = targetPos - currentPos;
    
    // Apply dead zone around target position to eliminate hunting
    if (abs(error) < DEADZONE) {
        velocity = 0;
        return currentPos;
    }
    
    // Adjust velocity based on distance (accelerate or decelerate)
    if (error > 0) {
        velocity = min(velocity + acceleration, maxVelocity);
    } else if (error < 0) {
        velocity = max(velocity - acceleration, -maxVelocity);
    }
    
    // Calculate new position
    float newPosFloat = currentPos + velocity;
    int newPos = round(newPosFloat);
    
    // Ensure we don't overshoot the target
    if ((velocity > 0 && newPos > targetPos) || (velocity < 0 && newPos < targetPos)) {
        newPos = targetPos;
        velocity = 0;
    }
    
    return constrain(newPos, 0, 180);
}

void loop() {
    buttonState = digitalRead(buttonPin);
    pressureValue = analogRead(pressureSensorPin);
    unsigned long currentTime = millis();
    
    if (buttonState == HIGH) {
        // Manual Mode
        autoMode = false;
        digitalWrite(ledPin, LOW);
        
        if (currentTime - lastMoveTime >= updateInterval) {
            lastMoveTime = currentTime;
            
            for(int i = 0; i < 5; i++) {
                // Read potentiometer and convert to degree range
                int rawVal = analogRead(potPins[i]);
                int val = map(rawVal, 0, 1023, 0, 180);
                
                // Apply multi-stage filtering to potentiometer readings
                val = applyPotFilter(i, val);
                
                // Apply dead zone to potentiometer readings
                int currentPos = currentPositions[i];
                val = applyDeadZone(val, currentPos, DEADZONE);
                
                // Use velocity-based control for smoother motion
                int newPos = moveWithVelocityControl(i, val);
                
                // Only write to the servo if position has changed meaningfully
                if (abs(newPos - currentPos) >= 1) {
                    // Use microsecond precision for smoother motion
                    int pulseWidth = degreesToMicroseconds(newPos);
                    myservos[i].writeMicroseconds(pulseWidth);
                    currentPositions[i] = newPos;
                    previousPositions[i] = newPos;
                }
            }
        }
        
        updateDisplay();
    } else {
        // Auto Mode
        if (!autoMode) {
            Serial.println("Auto Mode Activated");
            posCount = 0;
            autoMode = true;
            recording = true;
            ready = false;
            waitingForYN = false;
            digitalWrite(ledPin, HIGH);
            
            // Reset velocities when entering auto mode
            for(int i = 0; i < 5; i++) {
                servoVelocity[i] = 0;
            }
        }

        if (recording) {
            if (currentTime - lastMoveTime >= updateInterval) {
                lastMoveTime = currentTime;
                
                for(int i = 0; i < 5; i++) {
                    // Read potentiometer and convert to degree range
                    int rawVal = analogRead(potPins[i]);
                    int val = map(rawVal, 0, 1023, 0, 180);
                    
                    // Apply multi-stage filtering to potentiometer readings
                    val = applyPotFilter(i, val);
                    
                    // Apply dead zone to potentiometer readings
                    int currentPos = currentPositions[i];
                    val = applyDeadZone(val, currentPos, DEADZONE);
                    
                    // Use velocity-based control for smoother motion
                    int newPos = moveWithVelocityControl(i, val);
                    
                    // Only write to the servo if position has changed meaningfully
                    if (abs(newPos - currentPos) >= 1) {
                        // Use microsecond precision for smoother motion
                        int pulseWidth = degreesToMicroseconds(newPos);
                        myservos[i].writeMicroseconds(pulseWidth);
                        currentPositions[i] = newPos;
                        previousPositions[i] = newPos;
                    }
                }
            }

            // Store positions when S is pressed
            if (digitalRead(sButton) == LOW && (currentTime - lastDebounceTime > debounceDelay)) {
                lastDebounceTime = currentTime;
                if (posCount < 10) {
                    for(int i = 0; i < 5; i++) {
                        storedPositions[i][posCount] = currentPositions[i];
                    }
                    posCount++;
                    Serial.print("Position stored: ");
                    Serial.println(posCount);
                }
            }

            // Switch selected servo
            if (digitalRead(yButton) == LOW && (currentTime - lastDebounceTime > debounceDelay)) {
                lastDebounceTime = currentTime;
                selectedServo = (selectedServo + 1) % 5;
                Serial.print("Selected servo: ");
                Serial.println(selectedServo + 1);
            }

            // Finish recording
            if (digitalRead(tButton) == LOW && (currentTime - lastDebounceTime > debounceDelay)) {
                lastDebounceTime = currentTime;
                recording = false;
                waitingForYN = true;  // Set flag to show Y/N prompt
                
                // Smoothly move to home
                moveToHome();
                Serial.println("Recording finished. Loop mode? Y/N");
            }
        }

        if (waitingForYN) {
            updateDisplay();  // Update display to show the Y/N prompt
            
            if (digitalRead(yButton) == LOW && (currentTime - lastDebounceTime > debounceDelay)) {
                lastDebounceTime = currentTime;
                executeLoop = true;
                waitingForYN = false;
                ready = true;
                runMode = "LOOP";
                Serial.println("Loop mode selected");
            } else if (digitalRead(nButton) == LOW && (currentTime - lastDebounceTime > debounceDelay)) {
                lastDebounceTime = currentTime;
                executeLoop = false;
                waitingForYN = false;
                ready = true;
                runMode = "SINGLE";
                Serial.println("Single run mode selected");
            }
        }

        if (ready && !waitingForYN) {
            // Check if we have stored positions
            if (posCount == 0) {
                Serial.println("No positions stored. Exiting auto mode.");
                autoMode = false;
                digitalWrite(ledPin, LOW);
                return;
            }
            
            do {
                for (int pos = 0; pos < posCount && !stopExecution; pos++) {
                    Serial.print("Moving to position: ");
                    Serial.println(pos + 1);
                    
                    // Reset start time for S-curve calculation
                    moveStartTime = millis();
                    
                    // Save starting positions
                    for(int servo = 0; servo < 5; servo++) {
                        previousPositions[servo] = currentPositions[servo];
                    }
                    
                    // Execute S-curve motion profile
                    bool movementComplete = false;
                    while (!movementComplete && !stopExecution) {
                        currentTime = millis();
                        unsigned long elapsedTime = currentTime - moveStartTime;
                        
                        // Check for quit button
                        if (digitalRead(qButton) == LOW && (currentTime - lastDebounceTime > debounceDelay)) {
                            lastDebounceTime = currentTime;
                            stopExecution = true;
                            break;
                        }
                        
                        if (elapsedTime >= delayBetweenSteps) {
                            // Motion complete
                            movementComplete = true;
                            
                            // Ensure final positions are exact with filtering
                            for(int servo = 0; servo < 5; servo++) {
                                int finalPos = applySavitzkyGolayFilter(servo, storedPositions[servo][pos]);
                                int pulseWidth = degreesToMicroseconds(finalPos);
                                myservos[servo].writeMicroseconds(pulseWidth);
                                currentPositions[servo] = finalPos;
                                previousPositions[servo] = finalPos;
                                servoVelocity[servo] = 0; // Reset velocity
                            }
                        } else {
                            // Calculate and apply S-curve motion for each servo
                            for(int servo = 0; servo < 5; servo++) {
                                moveServoSmooth(servo, storedPositions[servo][pos], elapsedTime, delayBetweenSteps);
                            }
                        }
                        
                        updateDisplay();
                        delay(updateInterval);
                    }
                    
                    if (stopExecution) break;
                    
                    // Dwell at position
                    delay(750); // Increased dwell time
                }
                
                if (stopExecution) {
                    moveToHome();
                    break;
                }
                
                // Check if we need to continue looping
                if (executeLoop) {
                    Serial.println("Loop complete, starting next iteration");
                    // Longer pause between loops
                    delay(1500);
                }
                
            } while (executeLoop && !stopExecution);
            
            if (!executeLoop || stopExecution) {
                Serial.println("Execution complete, returning to manual mode");
                autoMode = false;
                stopExecution = false;
                digitalWrite(ledPin, LOW);
                runMode = "";
            }
        }
        
        updateDisplay();
    }
}

// Helper function to move all servos to home position
void moveToHome() {
    Serial.println("Moving to home position");
    
    // Reset start time for S-curve calculation
    moveStartTime = millis();
    
    // Save starting positions
    for(int i = 0; i < 5; i++) {
        previousPositions[i] = currentPositions[i];
        servoVelocity[i] = 0; // Reset velocities
    }
    
    // Execute S-curve motion profile
    bool movementComplete = false;
    while (!movementComplete) {
        unsigned long currentTime = millis();
        unsigned long elapsedTime = currentTime - moveStartTime;
        
        if (elapsedTime >= delayBetweenSteps) {
            // Motion complete
            movementComplete = true;
            
            // Ensure final positions are exact
            for(int i = 0; i < 5; i++) {
                int homePos = 0;
                myservos[i].writeMicroseconds(SERVO_MIN_PULSE);
                currentPositions[i] = homePos;
                previousPositions[i] = homePos;
            }
        } else {
            // Calculate and apply S-curve motion for each servo
            for(int i = 0; i < 5; i++) {
                moveServoSmooth(i, 0, elapsedTime, delayBetweenSteps);
            }
        }
        
        updateDisplay();
        delay(updateInterval);
    }
    
    updateDisplay();
}