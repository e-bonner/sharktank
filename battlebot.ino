#include <Adafruit_MotorShield.h>

#define MOTOR_L_PORT 1
#define MOTOR_R_PORT 2
#define CH_F_R_PIN 5
#define CH_L_R_PIN 4
#define CALC_MAX 512 // Max value for a channel in calculations
#define L_R_MAX 512 // Max change for L/R control
#define CH_MAX_F_R 1790
#define CH_MIN_F_R 1150
#define CH_MAX_L_R 1850
#define CH_MIN_L_R 1050
#define DEADBAND 75
#define DEADBAND_L_R 200

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *motorL = AFMS.getMotor(MOTOR_L_PORT);
Adafruit_DCMotor *motorR = AFMS.getMotor(MOTOR_R_PORT);

void setMotor(int16_t input, Adafruit_DCMotor *motor)
{
    input = constrain(input, -CALC_MAX, CALC_MAX);
    uint8_t direction;
    if (input < 0) {
        direction = BACKWARD;
        input *= -1;
    } else {
        direction = FORWARD;
    }
    input = map(input, 0, CALC_MAX, 0, 255);
    motor->setSpeed(input);
    motor->run(direction);
}

void setup()
{
    AFMS.begin();
    Serial.begin(9600);
    pinMode(CH_F_R_PIN, INPUT);
    pinMode(CH_L_R_PIN, INPUT);
}

void loop()
{
    int16_t power_fwd = map(pulseIn(CH_F_R_PIN, HIGH), CH_MIN_F_R, CH_MAX_F_R, -CALC_MAX, CALC_MAX);
    int16_t l_r = map(pulseIn(CH_L_R_PIN, HIGH), CH_MIN_L_R, CH_MAX_L_R, -L_R_MAX, L_R_MAX);
    if (abs(power_fwd) < DEADBAND && abs(l_r) < DEADBAND_L_R) {
        motorL->run(RELEASE);
        motorR->run(RELEASE);
        return;
    }
    float fwd = (float)power_fwd / CALC_MAX;
    fwd *= fabs(fwd); // Use quadratic control
    float l_r_fl = (float)l_r / CALC_MAX;
    l_r_fl *= fabs(l_r_fl);
    // int16_t l = power_fwd + l_r;
    int16_t l = (fwd - l_r_fl) * CALC_MAX;
    Serial.print(l);
    Serial.print(" ");
    // int16_t r = power_fwd - l_r;
    int16_t r = (fwd + l_r_fl) * CALC_MAX;
    Serial.println(r);
    setMotor(l, motorL);
    setMotor(r, motorR);
}