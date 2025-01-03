// SSR1-P_TCode_ESP32_Beta1
// by TempestMAx 20-11-2024
// Please copy, share, learn, innovate, give attribution.
// Decodes T-code commands and uses them to control servos a single brushless
// motor It can handle:
//   3x linear channels (L0, L1, L2)
//   3x rotation channels (R0, R1, R2)
//   3x vibration channels (V0, V1, V2)
//   3x auxilliary channels (A0, A1, A2)
// This code is designed to drive the SSR1 stroker robot, but is also intended
// to be used as a template to be adapted to run other t-code controlled arduino
// projects Have fun, play safe! History: Alpha1 - First release. 2-2-2023
// Alpha2 - Encoder moved to PIN33, End switch pin removed and start sequence
// changed. 23-2-2023 Alpha3 - SPI feedback by default added. PWM feedback
// switched to the native SFOC function. "Magic numbers" moved to the top of the
// code.
//          Code extensively tided up and comments added/improved. 16-5-2023
// Alpha4 - SSI feedback from MT6701 now default, but AS5048a still supported.
// 28-6-2023 Beta1 - Switch to ESP32-S3-Zero, all pins changed. Only MT6701
// tested so far! 20-11-2024

// ----------------------------
//  User Settings
// ----------------------------
// These are the setup parameters for the SSR1 on an ESP32

// Device IDs, for external reference
#define FIRMWARE_ID \
    "SSR1-P_TCode_ESP32_Beta1.ino"  // Device and firmware version
#define TCODE_VER "TCode v0.3"      // Current version of TCode

// Set to 00000000 for no PCB
#define PCB_VERSION 000102000

#define Encoder_PWM_PIN -1  // PWM feedback pin (if used) - P pad on AS5048a
#define ChipSelect_PIN \
    5  // SPI chip select pin - CSn on AS5048a (By default on ESP32-S3: MISO =
        // D13, MOSI = D11, CLK = D12)
#define Enable_PIN 4       // Motor enable - EN on SFOCMini
#define PWMchannel1_PIN 16  // Motor channel 1 - IN1 on SFOCMini
#define PWMchannel2_PIN 17  // Motor channel 2 - IN2 on SFOCMini
#define PWMchannel3_PIN 19  // Motor channel 3 - IN3 on SFOCMini
// HACK FOR PCB v1.2
#define SSIData_PIN 23
#define SSIClock_PIN 18

// Drive Parameters
#define MotorA_Voltage 20  // Motor operating voltage (12-20V)
#define MotorA_Current 1   // Maximum operating current (Amps)
// The control code needs to know the angle of the motor relative to the encoder
// - "Zero elec. angle". If a value is not entered it will perform a quick
// operation on startup to estimate this. This will be displayed in the serial
// monitor each time the device starts up. If the device is noticably faster in
// one direction the angle is out of alignment, try increasing or decreasing it
// by small increments (eg +/- 0.1).
#define MotorA_ParametersKnown \
    false  // Once you know the zero elec angle for the motor enter it below and
           // set this flag to true.
#define MotorA_ZeroElecAngle \
    2.75  // This number is the zero angle (in radians) for the motor relative
          // to the encoder.
#define MotorA_SensorDirection \
    Direction::CW  // Do not change. If the motor is showing CCW rotate the
                   // motor connector 180 degrees to reverse the motor.
#define SensorA_UseMT6701 \
    true  // Use the MT6701 encoder via SSI rather than the default AS5048a
          // encoder via SPI.
#define SensorA_UsePWM \
    false  // SPI feedback on the AS5048a is default because it's a lot smoother
           // and quieter! Change this to true if you want to use PWM feedback.

// Control constants
// (a.k.a. magic numbers for Eve)
#define RAIL_LENGTH 125          // Physical maximum travel of the receiver (mm)
#define STROKE_LENGTH 120        // Operating stroke length (mm)
#define PULLEY_CIRCUMFERENCE 60  // Drive pulley circumference (mm)
#define P_CONST 0.002            // Motor PID proportional constant
#define LOW_PASS \
    0.8  // Low pass filter factor for static noise reduction ( number < 1, 0 =
         // none)
// Derived constants
#define ANG_TO_POS                   \
    (10000 * PULLEY_CIRCUMFERENCE) / \
        (2 * 3.14159 * STROKE_LENGTH)  // Number to convert a motor angle to a
                                       // 0-10000 axis position
#define START_OFFSET                              \
    2 * 3.14159 * (RAIL_LENGTH - STROKE_LENGTH) / \
        (2 *                                      \
         PULLEY_CIRCUMFERENCE)  // Offset angle from endstop on startup (rad)

// Other functions
#define MIN_SMOOTH_INTERVAL \
    3  // Minimum auto-smooth ramp interval for live commands (ms)
#define MAX_SMOOTH_INTERVAL \
    100  // Maximum auto-smooth ramp interval for live commands (ms)

// T-Code Channels
#define CHANNELS 3  // Number of channels of each type (LRVA)

// Libraries used
#include <EEPROM.h>     // Permanent memory
#include <SimpleFOC.h>  // Motor controller code

#include <SimpleFOCDrivers.h>
#include <encoders/MT6701/MagneticSensorMT6701SSI.h>

// -----------------------------
// Class to handle each axis
// -----------------------------
class Axis {
   public:
    // Setup function
    Axis() {
        // Set default dynamic parameters
        rampStartTime = 0;
        rampStart = 5000;
        rampStopTime = rampStart;
        rampStop = rampStart;

        // Set Empty Name
        Name = "";
        lastT = 0;

        // Live command auto-smooth
        minInterval = MAX_SMOOTH_INTERVAL;
    }

    // Function to set the axis dynamic parameters
    void Set(int x, char ext, long y) {
        unsigned long t = millis();  // This is the time now
        x = constrain(x, 0, 9999);
        y = constrain(y, 0, 9999999);
        // Set ramp parameters, based on inputs
        // Live command
        if (y == 0 || (ext != 'S' && ext != 'I')) {
            // update auto-smooth regulator
            int lastInterval = t - rampStartTime;
            if (lastInterval > minInterval &&
                minInterval < MAX_SMOOTH_INTERVAL) {
                minInterval += 1;
            } else if (lastInterval < minInterval &&
                       minInterval > MIN_SMOOTH_INTERVAL) {
                minInterval -= 1;
            }
            // Set ramp parameters
            rampStart = GetPosition();
            rampStopTime = t + minInterval;
        }
        // Speed command
        else if (ext == 'S') {
            rampStart = GetPosition();  // Start from current position
            int d = x - rampStart;      // Distance to move
            if (d < 0) {
                d = -d;
            }
            long dt = d;  // Time interval (time = dist/speed)
            dt *= 100;
            dt /= y;
            rampStopTime = t + dt;  // Time to arrive at new position
            // if (rampStopTime < t + minInterval) { rampStopTime = t +
            // minInterval; }
        }
        // Interval command
        else if (ext == 'I') {
            rampStart = GetPosition();  // Start from current position
            rampStopTime = t + y;       // Time to arrive at new position
            // if (rampStopTime < t + minInterval) { rampStopTime = t +
            // minInterval; }
        }
        rampStartTime = t;
        rampStop = x;
        lastT = t;
    }

    // Function to return the current position of this axis
    int GetPosition() {
        int x;  // This is the current axis position, 0-9999
        unsigned long t = millis();
        if (t > rampStopTime) {
            x = rampStop;
        } else if (t > rampStartTime) {
            x = map(t, rampStartTime, rampStopTime, rampStart, rampStop);
        } else {
            x = rampStart;
        }
        x = constrain(x, 0, 9999);
        return x;
    }

    // Function to stop axis movement at current position
    void Stop() {
        unsigned long t = millis();  // This is the time now
        rampStart = GetPosition();
        rampStartTime = t;
        rampStop = rampStart;
        rampStopTime = t;
    }

    // Public variables
    String Name;          // Function name of this axis
    unsigned long lastT;  //

   private:
    // Movement positions
    int rampStart;
    unsigned long rampStartTime;
    int rampStop;
    unsigned long rampStopTime;

    // Live command auto-smooth regulator
    int minInterval;
};

// -----------------------------
// Class to manage Toy Comms
// -----------------------------
class TCode {
   public:
    // Setup function
    TCode(String firmware, String tcode) {
        firmwareID = firmware;
        tcodeID = tcode;
        halted = false;

        // Vibe channels start at 0
        for (int i = 0; i < CHANNELS; i++) {
            Vibration[i].Set(0, ' ', 0);
        }
    }

    // Function to name and activate axis
    void RegisterAxis(String ID, String axisName) {
        char type = ID.charAt(0);
        int channel = ID.charAt(1) - '0';
        if ((0 <= channel && channel < CHANNELS)) {
            switch (type) {
                // Axis commands
                case 'L':
                    Linear[channel].Name = axisName;
                    break;
                case 'R':
                    Rotation[channel].Name = axisName;
                    break;
                case 'V':
                    Vibration[channel].Name = axisName;
                    break;
                case 'A':
                    Auxiliary[channel].Name = axisName;
                    break;
            }
        }
    }

    // Function to read off individual bytes as input
    void ByteInput(byte inByte) {
        bufferString += (char)inByte;  // Add new character to string

        if (inByte == '\n') {             // Execute string on newline
            bufferString.trim();          // Remove spaces, etc, from buffer
            executeString(bufferString);  // Execute string
            bufferString = "";            // Clear input string
        }
    }

    // Function to read off whole strings as input
    void StringInput(String inString) {
        bufferString = inString;  // Replace existing buffer with input string
        bufferString.trim();      // Remove spaces, etc, from buffer
        executeString(bufferString);  // Execute string
        bufferString = "";            // Clear input string
    }

    // Function to set an axis
    void AxisInput(String ID, int magnitude, char extension,
                   long extMagnitude) {
        char type = ID.charAt(0);
        int channel = ID.charAt(1) - '0';
        if ((0 <= channel && channel < CHANNELS)) {
            switch (type) {
                // Axis commands
                case 'L':
                    Linear[channel].Set(magnitude, extension, extMagnitude);
                    break;
                case 'R':
                    Rotation[channel].Set(magnitude, extension, extMagnitude);
                    break;
                case 'V':
                    Vibration[channel].Set(magnitude, extension, extMagnitude);
                    break;
                case 'A':
                    Auxiliary[channel].Set(magnitude, extension, extMagnitude);
                    break;
            }
        }
    }

    // Function to read the current position of an axis
    int AxisRead(String ID) {
        int x = 5000;  // This is the return variable
        char type = ID.charAt(0);
        int channel = ID.charAt(1) - '0';
        if ((0 <= channel && channel < CHANNELS)) {
            switch (type) {
                // Axis commands
                case 'L':
                    x = Linear[channel].GetPosition();
                    break;
                case 'R':
                    x = Rotation[channel].GetPosition();
                    break;
                case 'V':
                    x = Vibration[channel].GetPosition();
                    break;
                case 'A':
                    x = Auxiliary[channel].GetPosition();
                    break;
            }
        }
        return x;
    }

    // Function to query when an axis was last commanded
    unsigned long AxisLast(String ID) {
        unsigned long t = 0;  // Return time
        char type = ID.charAt(0);
        int channel = ID.charAt(1) - '0';
        if ((0 <= channel && channel < CHANNELS)) {
            switch (type) {
                // Axis commands
                case 'L':
                    t = Linear[channel].lastT;
                    break;
                case 'R':
                    t = Rotation[channel].lastT;
                    break;
                case 'V':
                    t = Vibration[channel].lastT;
                    break;
                case 'A':
                    t = Auxiliary[channel].lastT;
                    break;
            }
        }
        return t;
    }

    bool isHalted()
    {
        return halted;
    }

   private:
    // Strings
    String firmwareID;
    String tcodeID;
    String bufferString;  // String to hold incomming commands
    bool halted; // Flag to signal motor control to disable

    // Declare axes
    Axis Linear[CHANNELS];
    Axis Rotation[CHANNELS];
    Axis Vibration[CHANNELS];
    Axis Auxiliary[CHANNELS];

    // Function to divide up and execute input string
    void executeString(String bufferString) {
        int index = bufferString.indexOf(' ');  // Look for spaces in string
        while (index > 0) {
            readCmd(
                bufferString.substring(0, index));  // Read off first command
            bufferString = bufferString.substring(
                index + 1);  // Remove first command from string
            bufferString.trim();
            index = bufferString.indexOf(' ');  // Look for next space
        }
        readCmd(bufferString);  // Read off last command
    }

    // Function to process the individual commands
    void readCmd(String command) {
        command.toUpperCase();

        // Switch between command types
        switch (command.charAt(0)) {
            // Axis commands
            case 'L':
            case 'R':
            case 'V':
            case 'A':
                axisCmd(command);
                break;

            // Device commands
            case 'D':
                deviceCmd(command);
                break;

            // Setup commands
            case '$':
                setupCmd(command);
                break;
        }
    }

    // Function to read and interpret axis commands
    void axisCmd(String command) {
        char type = command.charAt(0);  // Type of command - LRVA
        boolean valid = true;  // Command validity flag, valid by default

        // Check for channel number
        int channel = command.charAt(1) - '0';
        if (channel < 0 || channel >= CHANNELS) {
            valid = false;
        }
        channel = constrain(channel, 0, CHANNELS);

        // Check for an extension
        char extension = ' ';
        int index = command.indexOf('S', 2);
        if (index > 0) {
            extension = 'S';
        } else {
            index = command.indexOf('I', 2);
            if (index > 0) {
                extension = 'I';
            }
        }
        if (index < 0) {
            index = command.length();
        }

        // Get command magnitude
        String magString = command.substring(2, index);
        magString = magString.substring(0, 4);
        while (magString.length() < 4) {
            magString += '0';
        }
        int magnitude = magString.toInt();
        if (magnitude == 0 && magString.charAt(0) != '0') {
            valid = false;
        }  // Invalidate if zero returned, but not a number

        // Get extension magnitude
        long extMagnitude = 0;
        if (extension != ' ') {
            magString = command.substring(index + 1);
            magString = magString.substring(0, 8);
            extMagnitude = magString.toInt();
        }
        if (extMagnitude == 0) {
            extension = ' ';
        }

        // Switch between command types
        if (valid) {
            switch (type) {
                // Axis commands
                case 'L':
                    Linear[channel].Set(magnitude, extension, extMagnitude);
                    break;
                case 'R':
                    Rotation[channel].Set(magnitude, extension, extMagnitude);
                    break;
                case 'V':
                    Vibration[channel].Set(magnitude, extension, extMagnitude);
                    break;
                case 'A':
                    Auxiliary[channel].Set(magnitude, extension, extMagnitude);
                    break;
            }
        }
    }

    // Function to identify and execute device commands
    void deviceCmd(String command) {
        int i;
        // Remove "D"
        command = command.substring(1);

        // Look for device stop command
        if (command.substring(0, 4) == "STOP") {
            for (i = 0; i < 10; i++) {
                Linear[i].Stop();
            }
            for (i = 0; i < 10; i++) {
                Rotation[i].Stop();
            }
            for (i = 0; i < 10; i++) {
                Vibration[i].Set(0, ' ', 0);
            }
            for (i = 0; i < 10; i++) {
                Auxiliary[i].Stop();
            }
            halted = true;
        } else {
            // Look for numbered device commands
            int commandNumber = command.toInt();
            if (commandNumber == 0 && command.charAt(0) != '0') {
                command = -1;
            }
            switch (commandNumber) {
                case 0:
                    Serial.println(firmwareID);
                    break;

                case 1:
                    Serial.println(tcodeID);
                    break;

                case 2:
                    for (i = 0; i < 10; i++) {
                        axisRow("L" + String(i), 8 * i, Linear[i].Name);
                    }
                    for (i = 0; i < 10; i++) {
                        axisRow("R" + String(i), 8 * i + 80, Rotation[i].Name);
                    }
                    for (i = 0; i < 10; i++) {
                        axisRow("V" + String(i), 8 * i + 160,
                                Vibration[i].Name);
                    }
                    for (i = 0; i < 10; i++) {
                        axisRow("A" + String(i), 8 * i + 240,
                                Auxiliary[i].Name);
                    }
                    break;
            }
        }
    }

    // Function to modify axis preference values
    void setupCmd(String command) {
        int minVal, maxVal;
        String minValString, maxValString;
        boolean valid;
        // Axis type
        char type = command.charAt(1);
        switch (type) {
            case 'L':
            case 'R':
            case 'V':
            case 'A':
                valid = true;
                break;

            default:
                type = ' ';
                valid = false;
                break;
        }
        // Axis channel number
        int channel = (command.substring(2, 3)).toInt();
        if (channel == 0 && command.charAt(2) != '0') {
            valid = false;
        }
        // Input numbers
        int index1 = command.indexOf('-');
        if (index1 != 3) {
            valid = false;
        }
        int index2 =
            command.indexOf('-', index1 + 1);  // Look for spaces in string
        if (index2 <= 3) {
            valid = false;
        }
        if (valid) {
            // Min value
            minValString = command.substring(4, index2);
            minValString = minValString.substring(0, 4);
            while (minValString.length() < 4) {
                minValString += '0';
            }
            minVal = minValString.toInt();
            if (minVal == 0 && minValString.charAt(0) != '0') {
                valid = false;
            }
            // Max value
            maxValString = command.substring(index2 + 1);
            maxValString = maxValString.substring(0, 4);
            while (maxValString.length() < 4) {
                maxValString += '0';
            }
            maxVal = maxValString.toInt();
            if (maxVal == 0 && maxValString.charAt(0) != '0') {
                valid = false;
            }
        }
        // If a valid command, save axis preferences to EEPROM
        if (valid) {
            int memIndex = 0;
            switch (type) {
                case 'L':
                    memIndex = 0;
                    break;
                case 'R':
                    memIndex = 80;
                    break;
                case 'V':
                    memIndex = 160;
                    break;
                case 'A':
                    memIndex = 240;
                    break;
            }
            memIndex += 8 * channel;
            minVal = constrain(minVal, 0, 9999);
            EEPROM.put(memIndex, minVal - 1);
            minVal = constrain(maxVal, 0, 9999);
            EEPROM.put(memIndex + 4, maxVal - 10000);
            // Output that axis changed successfully
            switch (type) {
                case 'L':
                    axisRow("L" + String(channel), memIndex,
                            Linear[channel].Name);
                    break;
                case 'R':
                    axisRow("R" + String(channel), memIndex,
                            Rotation[channel].Name);
                    break;
                case 'V':
                    axisRow("V" + String(channel), memIndex,
                            Vibration[channel].Name);
                    break;
                case 'A':
                    axisRow("A" + String(channel), memIndex,
                            Auxiliary[channel].Name);
                    break;
            }
        }
    }

    // Function to print the details of an axis
    void axisRow(String axisID, int memIndex, String axisName) {
        int low, high;
        if (axisName != "") {
            EEPROM.get(memIndex, low);
            low = constrain(low, -1, 9998);
            EEPROM.get(memIndex + 4, high);
            high = constrain(high, -10000, -1);
            Serial.print(axisID);
            Serial.print(" ");
            Serial.print(low + 1);
            Serial.print(" ");
            Serial.print(high + 10000);
            Serial.print(" ");
            Serial.println(axisName);
        }
    }
};

// Done with generic TCode stuff...

// ----------------------------
//   SETUP
// ----------------------------
// This code runs once, on startup

// Declare classes
// This uses the t-code object above
TCode tcode(FIRMWARE_ID, TCODE_VER);

// encoder position monitor variables
volatile int encoderPulseLength = 464;
volatile int encoderPulseCycle = 920;
volatile int encoderPulseStart = 0;
// range is 5-928
volatile int longest = 500;
volatile int shortest = 500;

// Declare an SSI, PWM and SPI sensor. Only one will be used.
MagneticSensorMT6701SSI sensorA = MagneticSensorMT6701SSI(ChipSelect_PIN);
MagneticSensorPWM sensorA1 = MagneticSensorPWM(Encoder_PWM_PIN, 5, 928);
MagneticSensorSPI sensorA2 = MagneticSensorSPI(ChipSelect_PIN, 14, 0x3FFF);
// BLDC motor & driver instance
BLDCMotor motorA = BLDCMotor(11, 11.1);
BLDCDriver3PWM driverA = BLDCDriver3PWM(PWMchannel1_PIN, PWMchannel2_PIN,
                                        PWMchannel3_PIN, Enable_PIN);

// Position variables
int xLin;
float zeroAngle;
float xPosition;
float mode;
long startTime;
float motorVoltage;

// Tempest's troubleshooting code -IGNORE!
unsigned long previousMillis =
    0;                     // variable to store the time of the last report
const long interval = 10;  // interval at which to send reports (in ms)
int counter = 0;

SPIClass vspi(VSPI);

// Setup function
// This is run once, when the arduino starts
void setup() {
    // Start serial connection and report status
    Serial.begin(115200);
    tcode.StringInput("D0");
    tcode.StringInput("D1");

    // #ESP32# Enable EEPROM
    EEPROM.begin(320);

    // Register device axes
    tcode.RegisterAxis("L0", "Up");

    // Set Starting state
    zeroAngle = 0;
    mode = 0;
    
    // initialise encoder hardware
    if (SensorA_UseMT6701) {
        // Remap SPI pins for HWv1.2
        vspi.setFrequency(1000000);
        vspi.setDataMode(SPI_MODE2);
        vspi.setBitOrder(SPI_MSBFIRST);
        vspi.begin(SSIClock_PIN, SSIData_PIN, -1, -1);
        sensorA.init(&vspi);
    } else if (SensorA_UsePWM) {
        sensorA1.init();
    } else {
        sensorA2.init();
    }

    // driver config
    // power supply voltage [V]
    driverA.voltage_power_supply = MotorA_Voltage;
    // Max DC voltage allowed - default voltage_power_supply
    driverA.voltage_limit = 20;
    // driver init
    driverA.init();

    // limiting motor movements
    motorA.current_limit = MotorA_Current;  // [Amps]

    // set control loop type to be used
    motorA.torque_controller = TorqueControlType::voltage;
    motorA.controller = MotionControlType::torque;

    // link the motor to the sensor
    if (SensorA_UsePWM) {
        motorA.linkSensor(&sensorA1);
    } else {
        motorA.linkSensor(&sensorA);
    }
    // link the motor and the driver
    motorA.linkDriver(&driverA);

    // initialize motor
    motorA.init();
    if (MotorA_ParametersKnown) {
        motorA.sensor_direction = MotorA_SensorDirection;
        motorA.zero_electric_angle = MotorA_ZeroElecAngle;  // rad
    } else {
        motorA.useMonitoring(Serial);
    }
    motorA.initFOC();

    // Set sensor angle and pre-set zero angle to current angle
    if (SensorA_UseMT6701) {
        sensorA.update();
        zeroAngle = sensorA.getSensorAngle();
    } else if (SensorA_UsePWM) {
        sensorA1.update();
        zeroAngle = sensorA1.getAngle();
    } else {
        sensorA2.update();
        zeroAngle = sensorA2.getAngle();
    }
    // Record start time
    startTime = millis();

    // Signal ready to start
    Serial.println("Ready!");
}

// ----------------------------
//   MAIN
// ----------------------------
// This loop runs continuously
void loop() {
    // Read serial and send to tcode class
    while (Serial.available() > 0) {
        // Send the serial bytes to the t-code object
        tcode.ByteInput(Serial.read());
    }

    // Run motor FOC loop
    motorA.loopFOC();

    // Collect inputs
    // These functions query the t-code object for the position/level at a
    // specified time Number recieved will be an integer, 0-9999
    xLin = tcode.AxisRead("L0");

    // Update sensor position
    float angle;
    if (SensorA_UseMT6701) {
        sensorA.update();
        angle = sensorA.getAngle();
    } else if (SensorA_UsePWM) {
        sensorA1.update();
        angle = sensorA1.getAngle();
    } else {
        sensorA2.update();
        angle = sensorA2.getAngle();
    }
    // Determine the linear position of the receiver in (0-10000)
    xPosition = (angle - zeroAngle) * ANG_TO_POS;

    // Control by motor voltage
    float motorVoltageNew;
    // If the device is in startup mode roll downwards for two seconds and press
    // against bottom stop. Distance of travel is 12,000 (>10,000) just to make
    // sure that the receiver reaches the bottom.
    if (mode == 0) {
        xLin = map(millis() - startTime, 0, 2000, 0, -12000);
        if (millis() > (startTime + 2000)) {
            mode = 1;
            zeroAngle = angle + START_OFFSET;
        }
        motorVoltageNew = P_CONST * (xLin - xPosition);
        if (motorVoltageNew < -0.5) {
            motorVoltageNew = -0.5;
        }
        // Otherwise set motor voltage based on position error
    } else {
        motorVoltageNew = P_CONST * (xLin - xPosition);
    }
    // Low pass filter to reduce motor noise
    motorVoltage = LOW_PASS * motorVoltage + (1 - LOW_PASS) * motorVoltageNew;
    
    if (tcode.isHalted())
    {
        motorA.disable();
    }
    else
    {
        motorA.move(motorVoltage);
    }
}
