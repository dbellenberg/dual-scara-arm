// Define the pin connected to the actuator PIN IS D9
#define ACTUATOR_PIN 9

void setup() {
  // Initialize the ACTUATOR_PIN as an output
  pinMode(ACTUATOR_PIN, OUTPUT);
}

void loop() {
  // Turn the actuator on
  digitalWrite(ACTUATOR_PIN, HIGH);
  // Wait for 2 seconds
  delay(1000);
  // Turn the actuator off
  digitalWrite(ACTUATOR_PIN, LOW);
  // Wait for 2 seconds
  delay(1000);
}


bool actuator_high = false;

void actuator_up() { 
    if (actuator_high == false) {
        digitalWrite(ACTUATOR_PIN, HIGH);
        actuator_high = true;
    }
}

void actuator_down() {
    if (actuator_high == true) {
        digitalWrite(ACTUATOR_PIN, LOW);
        actuator_high = false;
    }
}


// Path: arduino_code/solenoid_linear_actuator.ino

// Define the pin connected to the solenoid
#define MAGNET_SOLENOID_PIN 10

void setup() {
  // Initialize the SOLENOID_PIN as an output
  pinMode(SOLENOID_PIN, OUTPUT);
}

void loop() {
  // Turn the solenoid on
  digitalWrite(SOLENOID_PIN, HIGH);
  // Wait for 1 second
  delay(2000);
  // Turn the solenoid off
  digitalWrite(SOLENOID_PIN, LOW);
  // Wait for 1 second
  delay(2000);
}

bool magnet_on = false;

void magnet_on() { 
    if (magnet_on == false) {
        digitalWrite(SOLENOID_PIN, HIGH);
        magnet_on = true;
    }
}

void magnet_off() {
    if (magnet_on == true) {
        digitalWrite(SOLENOID_PIN, LOW);
        magnet_on = false;
    }
}