// IMPORT PACKAGES
#include <AccelStepper.h> // for stepper motor control
#include <MultiStepper.h> // for multiple stepper motor control
#include <math.h>  // for trigonometric functions

// Define step and direction pins used by the RAMPS shield for X-axis
#define X_STEP_PIN 54 // X_STEP_PIN on RAMPS 1.4
#define X_DIR_PIN 55  // X_DIR_PIN on RAMPS 1.4
#define X_ENABLE_PIN 38 // X_ENABLE_PIN on RAMPS 1.4

// Define step and direction pins used by the RAMPS shield for Y-axis
#define Y_STEP_PIN 60 // Y_STEP_PIN on RAMPS 1.4
#define Y_DIR_PIN 61  // Y_DIR_PIN on RAMPS 1.4
#define Y_ENABLE_PIN 56 // Y_ENABLE_PIN on RAMPS 1.4

// Define the switch pin
#define HOME_SWITCH 3 // Pin 3 connected to Click Switch

// Define the pin connected to the actuator PIN IS D9
#define LINEAR_ACTUATOR_PIN 9

// Define the pin connected to the solenoid PIN IS D10
#define MAGNET_SOLENOID_PIN 10

int current_steps_right;
int current_steps_left;

// Define fixed parameters
const int steps_per_revolution = 3200; //steps per revolution

// Define the variables
double x = 0; // X-coordinate
double y = 0; // Y-coordinate
double q1; // Angle of the first arm
double q2; // Angle of the second arm

struct Point {
    double x;
    double y;
};


// Define the stepper motor connections
AccelStepper stepper1(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

//Instantiate MultiStepper
MultiStepper steppers;

// BOOLEAN VARIABLES
bool motorsEnabled = false; // Flag to enable/disable the motors
bool start_loop = false;  // Flag to indicate if the loop should start
bool actuator_high = false; // Flag to indicate if the actuator is high
bool magnet_active = false; // Flag to indicate if the magnet is on

// Function to set the current positions to zero when the switch is pressed
void setManualPosition() {
  // Read the state of the switch
  int switchState = digitalRead(HOME_SWITCH);
  
  // Check if the switch is clicked (pressed)
  if (switchState == LOW) {
    // If the switch is clicked, set the current positions as zero
    stepper1.setCurrentPosition(800);
    stepper2.setCurrentPosition(800);

    Serial.print("\n");

   

    // Set the flag to start the loop
    start_loop = true;

    enableMotors();

    delay(1000);
    moveTo(60,300);
    
  }
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

    // Update the current position
    x = new_x;
    y = new_y;
}

// Function to calculate the inverse kinematics
void moveToNew(Point* positions, int count) {
  for (int i = 0; i < count; i++) {
      Point position = positions[i];
      float angle_left_new;
      float angle_right_new;

      angle_left_new = calculateAngleLeft(position.x, position.y);
      angle_right_new = calculateAngleRight(position.x - OFFSET, position.y);

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

      // Update the current position
      // Update current position
      x = position.x;
      y = position.y;
    }
}

void moveIP(double targetX, double targetY) {
    const int MAX_POINTS = 100; // Adjust this based on your needs and memory constraints
    Point positions[MAX_POINTS];
    int numPositions = 0;

    double stepLength = 150; // Step length in millimeters
    double distance = sqrt(pow(targetX - x, 2) + pow(targetY - y, 2));
    int numSteps = ceil(distance / stepLength);
    double stepX = (targetX - x) / numSteps;
    double stepY = (targetY - y) / numSteps;

    for (int step = 0; step <= numSteps; step++) {
        if (numPositions < MAX_POINTS) {
            positions[numPositions].x = x + stepX * step;
            positions[numPositions].y = y + stepY * step;
            numPositions++;
        }
    }

    moveToNew(positions, numPositions);
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

void draw_rectangle() {  
  moveTo(-200,200);
  moveTo(300,200);
  //moveTo(200,200);
  //moveTo(0,200);

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

//INTERPOLATION TEST
void draw_line_test() {

  int steps = 10;
  for (int step = 0; step <= steps; step++) {
      moveIP(0,150);
      moveIP(200,300);
    }
}

// Linear Actuator Functions
void actuator_up() { 
    Serial.print("Actuator up");
    digitalWrite(LINEAR_ACTUATOR_PIN, LOW);
}

void actuator_down() {
    Serial.print("Actuator down");
    digitalWrite(LINEAR_ACTUATOR_PIN, HIGH);    
}

// Magnet Functions
void magnet_on() { 
    if (magnet_active == false) {
        digitalWrite(MAGNET_SOLENOID_PIN, HIGH);
        magnet_active = true;
    }
}

void magnet_off() {
    if (magnet_active == true) {
        digitalWrite(MAGNET_SOLENOID_PIN, LOW);
        magnet_active = false;
    }
}

// Pick up the object
void pick_up() {
    // Move the arm down
    actuator_down();
    // Turn on the magnet
    magnet_on();
    //delay
    delay(1000);
    // Move the arm up
    actuator_up();
}

// Drop the object
void place() {
    // Move the arm down
    actuator_down();
    //delay
    delay(1000);
    // Turn off the magnet
    magnet_off();
    // Move the arm up
    actuator_up();
}


void demo() {
    delay(1000);
    //pick and place
    moveIP(60,200);
    pick_up();
    moveIP(-250,200);
    place();
    //pick and place
    moveIP(200,100);
    pick_up();
    moveIP(-250,200);
    place();
    //pick and place
    moveIP(200,400);
    pick_up();
    moveIP(-250,200);
    place();
    //pick and place
    moveIP(0,350);
    pick_up();
    moveIP(-250,200);
    place();
    //pick and place
    moveIP(-75,100);
    pick_up();
    moveIP(-250,200);
    place();
    moveIP(60,300);
}

void setup() {
  Serial.begin(9600);

  // Set the maximum speed and acceleration for X-axis motor
  stepper1.setMaxSpeed(350); // steps per second
  stepper1.setAcceleration(50); // steps per second per second

  // Set the maximum speed and acceleration for Y-axis motor
  stepper2.setMaxSpeed(350); // steps per second
  stepper2.setAcceleration(50); // steps per second per second

  // Enable the stepper drivers
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  
  //disableMotors();
  // Add to MultiStepper
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);

  pinMode(HOME_SWITCH, INPUT_PULLUP); // Set home switch
  pinMode(LINEAR_ACTUATOR_PIN, OUTPUT); // Set Linear Actuator Pin
  pinMode(MAGNET_SOLENOID_PIN, OUTPUT); // Set Linear Actuator Pin
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
            float xPosition = xString.toDouble();
            float yPosition = yString.toDouble();

            // Move the robot arm to the specified position
            moveIP(xPosition, yPosition);

            // Optionally, print out the position for confirmation
            Serial.print("Moving to position X: ");
            Serial.print(xPosition);
            Serial.print(", Y: ");
            Serial.println(yPosition);
        } 

        else if (inputString == "rectangle") {
          Serial.print("Perform rectangle");
          for (int i = 0; i <= 10; i = i + 1) {
            draw_rectangle();
            }
        } 

        else if (inputString == "line") {
          Serial.print("Perform line");
          draw_line_test();
        }

        else if (inputString == "actuator_up") {
          actuator_up();
        }

        else if (inputString == "actuator_down") {
              Serial.print("Actuator down");
              actuator_down();
            }

        else if (inputString == "pick") {
              Serial.print("\n");
              Serial.print("Pick up");
              pick_up();
            }

        else if (inputString == "place") {
              Serial.print("\n");
              Serial.print("Place");
              place();
            }
        
        else if (inputString == "dem0") {
              Serial.print("\n");
              Serial.print("Demo");
              demo();
            }
            
        else {
              // Invalid input format
              Serial.print("\n");
              Serial.println("Invalid input. Please enter in format: X,Y");
            } 

    }
}
