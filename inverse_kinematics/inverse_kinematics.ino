#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>  // include cmath for trigonometric functions

// Define step and direction pins used by the RAMPS shield for X-axis
#define Y_STEP_PIN 54 // X_STEP_PIN on RAMPS 1.4
#define Y_DIR_PIN 55  // X_DIR_PIN on RAMPS 1.4
#define Y_ENABLE_PIN 38 // X_ENABLE_PIN on RAMPS 1.4

// Define step and direction pins used by the RAMPS shield for Y-axis
#define X_STEP_PIN 60 // Y_STEP_PIN on RAMPS 1.4
#define X_DIR_PIN 61  // Y_DIR_PIN on RAMPS 1.4
#define X_ENABLE_PIN 56 // Y_ENABLE_PIN on RAMPS 1.4

// Define the switch pin
#define home_switch 3 // Pin 3 connected to Click Switch

int current_steps_right;
int current_steps_left;

// Define fixed parameters
const double l1 = 22; // Length of the first arm
const double l2 = 27.3; // Length of the second arm
const int steps_per_revolution = 3200; //steps per revolution
double d = 12;

// Define the variables
double x = 0; // X-coordinate
double y = 0; // Y-coordinate
double q1; // Angle of the first arm
double q2; // Angle of the second arm

// Define the stepper motor connections
AccelStepper stepper1(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

//Instantiate MultiStepper
MultiStepper steppers;

bool motorsEnabled = false; // Flag to enable/disable the motors
bool start_loop = false;  // Flag to indicate if the loop should start

// Function to set the current positions to zero when the switch is pressed
void setManualPosition() {
  // Read the state of the switch
  int switchState = digitalRead(home_switch);
  
  // Check if the switch is clicked (pressed)
  if (switchState == LOW) {
    // If the switch is clicked, set the current positions as zero
    stepper1.setCurrentPosition(800);
    stepper2.setCurrentPosition(800);

    //current_steps_left = 800;
    //current_steps_right = 800;

    // Set the flag to start the loop
    start_loop = true;
  }
  enableMotors();
}


const double L1 = 220 /* Your L1 value here */;
const double L2 = 273 /* Your L2 value here */;
const double OFFSET = 120 /* Your OFFSET value here */;


double calculateAngleRight(double x, double y) {
    double r = sqrt(x * x + y * y);
    double cos_theta_2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    cos_theta_2 = fmax(-1.0, fmin(cos_theta_2, 1.0));
    double theta_2 = acos(cos_theta_2);

    double k1 = L1 + L2 * cos(theta_2);
    double k2 = L2 * sin(theta_2);
    double theta_1 = atan2(y, x) - atan2(k2, k1);

    return theta_1 * 180 / M_PI; // Convert to degrees
}

double calculateAngleLeft(double x, double y) {
    double r = sqrt(x * x + y * y);
    double cos_theta_2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    cos_theta_2 = fmax(-1.0, fmin(cos_theta_2, 1.0));
    double theta_2 = acos(cos_theta_2);
    theta_2 = -theta_2;

    double k1 = L1 + L2 * cos(theta_2);
    double k2 = L2 * sin(theta_2);
    double theta_1 = atan2(y, x) - atan2(k2, k1);

    return theta_1 * 180 / M_PI; // Convert to degrees
}

// Function to calculate the inverse kinematics
void moveTo(double new_x, float new_y){
    double diff_x = new_x - d;

    float B_left[2] = {new_x, new_y};
    float B_right[2] = {diff_x, new_y};
    
    float angleLeft;
    float angleRight;
    float C_left[2];
    float C_right[2];

    float angle_left_new;
    float angle_right_new;

    angle_left_new = calculateAngleLeft(new_x, new_y);
    angle_right_new = calculateAngleRight(new_x - OFFSET, new_y);


    // Print the calculated angles
    Serial.print("Angle Left");
    Serial.print("\n");
    Serial.print(angle_left_new);
    Serial.print("\n");
    Serial.print("\n");

    Serial.print("Angle Right");
    Serial.print("\n");
    Serial.print(angle_right_new);
    Serial.print("\n");
    Serial.print("\n");



    // calculate steps to move
    int left_steps = steps_per_revolution * (angle_left_new / 360);
    int right_steps = steps_per_revolution * (angle_right_new / 360);

    Serial.print("STEPS Left");
    Serial.print("\n");
    Serial.print(left_steps);
    Serial.print("\n");
    Serial.print("\n");

    Serial.print("STEPS Right");
    Serial.print("\n");
    Serial.print(right_steps);
    Serial.print("\n");
    Serial.print("\n");

    // move engines
    long positions[2]; // Array of desired stepper positions
    
    positions[0] = left_steps;
    positions[1] = right_steps;
    
    steppers.moveTo(positions);
    steppers.runSpeedToPosition(); // Blocks until all are in position
    //delay(1000);
}

// Function to enable/disable the motors
void enableMotors() {
    // If the motors are enabled, disable them
    digitalWrite(X_ENABLE_PIN, LOW); // Set enable HIGH to disable X-axis stepper driver
    digitalWrite(Y_ENABLE_PIN, LOW); // Set enable HIGH to disable Y-axis stepper driver

    // Set the flag to false
    motorsEnabled = true;
}

// Function to specifically disable the motors
void disableMotors() {
  // Disable the motors
  digitalWrite(X_ENABLE_PIN, HIGH); // Set enable HIGH to disable X-axis stepper driver
  digitalWrite(Y_ENABLE_PIN, HIGH); // Set enable HIGH to disable Y-axis stepper driver

  // Set the flag to false
  motorsEnabled = false;
}


void setup() {
  Serial.begin(9600);

  // Set the maximum speed and acceleration for X-axis motor
  stepper1.setMaxSpeed(1000); // steps per second
  stepper1.setAcceleration(250); // steps per second per second

  // Set the maximum speed and acceleration for Y-axis motor
  stepper2.setMaxSpeed(1000); // steps per second
  stepper2.setAcceleration(250); // steps per second per second

  // Enable the stepper drivers
  pinMode(X_ENABLE_PIN, OUTPUT);
  //digitalWrite(X_ENABLE_PIN, LOW); // Set enable LOW to enable X-axis stepper driver --> enable/disable function
  pinMode(Y_ENABLE_PIN, OUTPUT);
  //digitalWrite(Y_ENABLE_PIN, LOW); // Set enable LOW to enable Y-axis stepper driver -->  enable/disable function
  
  //disableMotors();
  // Add to MultiStepper
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);

  pinMode(home_switch, INPUT_PULLUP); // Set home switch
}

void draw_rectangle() {  
  moveTo(0,400);
  moveTo(120,400);
  moveTo(120,300);
  moveTo(0,300);

}

void draw_line_horizontal_old() {  
  moveTo(-300,200);
  moveTo(400,200);
}

//INTERPOLATION TEST
void draw_line_horizontal() {
    int startX = -300, startY = 200;
    int endX = 400, endY = 200;

    int steps = 25; // Number of steps for interpolation
    float deltaX = (float)(endX - startX) / steps;
    float deltaY = (float)(endY - startY) / steps;

    // Move forward from start to end
    for (int step = 0; step <= steps; step++) {
        int newX = startX + round(deltaX * step);
        int newY = startY + round(deltaY * step);
        moveTo(newX, newY);
    }

    // Move backward from end to start
    for (int step = steps; step >= 0; step--) {
        int newX = startX + round(deltaX * step);
        int newY = startY + round(deltaY * step);
        moveTo(newX, newY);
    }
}



void loop() {
    
    // Call the setManualPosition function to set the positions to zero if start_loop is not true
    if (!start_loop) {
      disableMotors();
      setManualPosition();
    }

    // Check if data is available to read from the serial port
    if (Serial.available() > 0) {
        // Read the incoming string until newline character
        String inputString = Serial.readStringUntil('\n');
        
        // Split the string at the comma to get x and y values
        int commaIndex = inputString.indexOf(',');
        if (commaIndex != -1) {
            // Extract x and y substrings
            String xString = inputString.substring(0, commaIndex);
            String yString = inputString.substring(commaIndex + 1);

            // Convert the substrings to float
            float xPosition = xString.toFloat();
            float yPosition = yString.toFloat();

            // Move the robot arm to the specified position
            moveTo(xPosition, yPosition);

            // Optionally, print out the position for confirmation
            Serial.print("Moving to position X: ");
            Serial.print(xPosition);
            Serial.print(", Y: ");
            Serial.println(yPosition);
        } else {

            if (inputString == "s") {
              long positions[2];
              positions[0] = -400;
              positions[1] = 400;
              steppers.moveTo(positions);

              steppers.runSpeedToPosition(); // Blocks until all are in position
            }

            if (inputString == "rectangle") {
              for (int i = 0; i <= 10; i = i + 1) {
                  draw_rectangle();
                  } 
            } 
            
            if (inputString == "line") {
              for (int i = 0; i <= 10; i = i + 1) {
                draw_line_horizontal();
                }
            }
            
            else {
                // Invalid input format
                Serial.println("Invalid input. Please enter in format: X,Y");
            } 

        }
    }
}
