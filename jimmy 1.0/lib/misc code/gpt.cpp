#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

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

// Variables
unsigned long last_update_time = 0;
double time = 0.0;
double gimbal_angle_pitch_deg = 0.0;
double gimbal_angle_yaw_deg = 0.0;
double pitch_angle = 0.0; // Pitch angle (rad)
double angular_velocity_pitch = 0.0; // Angular velocity from gyroscope

// State variables for positions and velocities
double accel_x = 0.0, accel_z = 0.0; // Accelerations in X and Z directions
double velocity_x = 0.0, velocity_z = 0.0; // Velocities in X and Z directions
double position_x = 0.0, position_z = 0.0; // Positions in X and Z directions

// PID state variables
double previous_error = 0.0;
double integral = 0.0;

void setup() {
    Serial.begin(115200);

    // Servo initialization
    servoPitch.attach(servoPitchPin);
    servoYaw.attach(servoYawPin);

    // MPU6050 initialization
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("Failed to initialize MPU6050!");
        while (1);
    }

    last_update_time = millis();
}

void loop() {
    unsigned long current_time = millis();
    if (current_time - last_update_time >= time_step * 1000) {
        last_update_time = current_time;

        // Read MPU6050 data
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // Convert accelerometer data (ax, ay, az) to tilt angle
        double tilt_angle = atan2(ay, az) * radian_conversion; // Tilt angle in radians

        // Convert gyroscope data to angular velocity (rad/s)
        angular_velocity_pitch = gy / 131.0 * radian_conversion;

        // Update pitch angle by integrating angular velocity
        pitch_angle += angular_velocity_pitch * time_step;

        // Define setpoint (desired pitch angle, in degrees)
        double setpoint = 10.0; // Desired pitch angle after 500m altitude
        if (position_z <= 500.0) {
            setpoint = 0.0; // Keep upright below 500m
        }

        // Calculate error for PID control (setpoint in radians)
        double error = (setpoint * radian_conversion) - pitch_angle;

        // PID calculations
        integral += error * time_step;
        double derivative = (error - previous_error) / time_step;
        double output = Kp * error + Ki * integral + Kd * derivative;

        // Cap output to gimbal angle limits
        if (output > 10.0) output = 10.0;
        if (output < -10.0) output = -10.0;

        gimbal_angle_pitch_deg = output;

        // Calculate forces in X and Z based on gimbal pitch angle
        double force_x = gimbal_force * cos(gimbal_angle_pitch_deg * radian_conversion);
        double force_z = gimbal_force * sin(gimbal_angle_pitch_deg * radian_conversion);

        // Calculate accelerations
        accel_x = force_x / mass;
        accel_z = force_z / mass;

        // Integrate accelerations to find velocities
        velocity_x += accel_x * time_step;
        velocity_z += accel_z * time_step;

        // Integrate velocities to find positions
        position_x += velocity_x * time_step;
        position_z += velocity_z * time_step;

        // Set servo angles
        servoPitch.write(map(gimbal_angle_pitch_deg, -10, 10, 0, 180));
        servoYaw.write(map(gimbal_angle_yaw_deg, -10, 10, 0, 180));

        // Print results
        Serial.print("Time: "); Serial.print(time);
        Serial.print(" s\tPitch Angle: "); Serial.print(pitch_angle * 180.0 / M_PI);
        Serial.print(" deg\tSetpoint: "); Serial.print(setpoint);
        Serial.print(" deg\tOutput: "); Serial.println(output);
        Serial.print("Tilt Angle: "); Serial.print(tilt_angle * 180.0 / M_PI);
        Serial.print(" deg\tPosition Z: "); Serial.println(position_z);

        // Save current error for next loop
        previous_error = error;

        // Increment time
        time += time_step;
    }
}