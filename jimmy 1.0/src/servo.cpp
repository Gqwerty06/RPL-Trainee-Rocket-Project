#include <Wire.h>
#include <Esp32Servo.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

// Constants
const double mass = 1.5;           // Rocket mass (kg)
const double inertia = 0.2868;     // Rocket moment of inertia
const double distance = 0.358;     // Distance from center to force application point (m)
const double gimbal_force = 60;    // Constant force applied by the gimbal (N)
const double time_step = 0.01;     // Simulation time step (s)
const double radian_conversion = M_PI / 180.0;

// PID constants
const double Kp = 1.0;  // Proportional gain
const double Ki = 0.1;  // Integral gain
const double Kd = 0.05; // Derivative gain

// Servo setup
Servo servoPitch;
Servo servoYaw;
const int servoPitchPin = 9; // Pin for pitch servo
const int servoYawPin = 10;  // Pin for yaw servo

// Sensors
MPU6050 mpu;
Adafruit_BMP3XX bmp;

// Variables
double gimbal_angle_pitch_deg = 0.0;
double gimbal_angle_yaw_deg = 0.0;
double angular_accel_pitch = 0.0;  // Angular acceleration about pitch axis
double angular_velocity_pitch = 0.0; // Angular velocity (rad/s)
double pitch_angle = 0.0; // Pitch angle (rad)
double accel_x = 0.0, accel_z = 0.0; // Accelerations in X and Z directions
double velocity_x = 0.0, velocity_z = 0.0; // Velocities in X and Z directions
double position_x = 0.0, position_z = 0.0; // Positions in X and Z directions
unsigned long last_update_time = 0;

// PID state variables
double previous_error = 0.0;
double integral = 0.0;
double timeCount = 0.0;
double altitude = 0.0;
double setpoint = 0.0; 

void setup() {
    Serial.begin(115200);

    // Servo initialization
    servoPitch.attach(servoPitchPin);
    servoYaw.attach(servoYawPin);

    last_update_time = millis();

    sensors_event_t a, g, temp;
}

void loop() {
    unsigned long current_time = millis();
    if (current_time - last_update_time >= time_step * 1000) {
        last_update_time = current_time;

       
        // Read sensors
        if (bmp.performReading()) {
            altitude = bmp.readAltitude(1013.25); // Standard sea level pressure
        }

        // Define setpoint (desired pitch angle, in degrees)
                if (altitude > 500.0) {
                    setpoint = 10.0; // Change to to above 500m
        }

        // Calculate angular acceleration for pitch
        double gimbal_angle_pitch_rad = gimbal_angle_pitch_deg * radian_conversion;
        angular_accel_pitch = (gimbal_force * distance * cos(gimbal_angle_pitch_rad)) / inertia;

        // Integrate angular acceleration to get angular velocity
        angular_velocity_pitch += angular_accel_pitch * time_step;

        // Integrate angular velocity to get pitch angle
        pitch_angle += angular_velocity_pitch * time_step;

        // Calculate error for PID control (setpoint in radians)
        double error = setpoint * radian_conversion - pitch_angle;

        // PID calculations
        integral += error * time_step;
        double derivative = (error - previous_error) / time_step;
        double output = Kp * error + Ki * integral + Kd * derivative;

        // Cap output to gimbal angle limits
        if (output > 10.0) output = 10.0;
        if (output < -10.0) output = -10.0;

        gimbal_angle_pitch_deg = output;

        // Calculate forces in X and Z
        double force_x = gimbal_force * cos(pitch_angle);
        double force_z = gimbal_force * sin(pitch_angle);

        // Calculate accelerations
        accel_x = force_x / mass;
        accel_z = force_z / mass;

        // Integrate to find velocities
        velocity_x += accel_x * time_step;
        velocity_z += accel_z * time_step;

        // Integrate to find positions
        position_x += velocity_x * time_step;
        position_z += velocity_z * time_step;

        // Set servo angles
        servoPitch.write(map(gimbal_angle_pitch_deg, -10, 10, 0, 180));
        servoYaw.write(map(gimbal_angle_yaw_deg, -10, 10, 0, 180));

        // Print results
        Serial.print("Time: "); 
        Serial.print(timeCount);
        Serial.print(" s\tAltitude: "); Serial.print(altitude);
        Serial.print(" m\tPitch Angle: "); Serial.print(pitch_angle * 180.0 / M_PI);
        Serial.print(" deg\tSetpoint: "); Serial.print(setpoint);
        Serial.print(" deg\tOutput: "); Serial.println(output);

        // Save current error for next loop
        previous_error = error;

        // Increment time
        timeCount += time_step;
    }
}