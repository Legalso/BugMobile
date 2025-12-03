// ========================================================
// === BUGMOBILE -
// ========================================================

#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>

// ========================================================
// >>>>>> CALIBRAÇÃO <<<<<<
// ========================================================

#define SPEED_MODIFIER    0.65

// Distâncias
#define DIST_WARN         35
#define DIST_STOP         18

// --- AJUSTE DO RADAR ---
#define SERVO_CENTER      90

// Abertura: 35 graus para cada lado (Total 70 graus)
#define SCAN_ANGLE_MIN    65
#define SCAN_ANGLE_MAX    115

// Fluidez
#define SCAN_STEP         10   
#define SCAN_INTERVAL     40   

// Giro
#define TURN_DELAY_90_DEG 690

// ========================================================

#define TRIG_PIN     A1
#define ECHO_PIN     A0
#define SERVO_PIN    10
#define MAX_DISTANCE 200

#define SPEED_CRUISE   (int)(180 * SPEED_MODIFIER)
#define SPEED_KICK     (int)(255 * SPEED_MODIFIER)
#define SPEED_TURN     (int)(255 * SPEED_MODIFIER)
#define KICK_TIME      150
#define RE             100

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
Servo headServo;

AF_DCMotor motorEsqFrente(1);
AF_DCMotor motorDirFrente(2);
AF_DCMotor motorEsqTras(3);
AF_DCMotor motorDirTras(4);

unsigned long lastScanTime = 0;
int currentServoPos = SERVO_CENTER;
int servoDirection = 1; 
bool wasStopped = true;

void setup() {
    Serial.begin(9600);
    Serial.println(">>> V6.8 <<<");
    headServo.attach(SERVO_PIN);
    headServo.write(SERVO_CENTER);
    stopMotors();
    delay(2000);
}

void loop() {
    int currentDist = updateRadar(); 
    if (currentDist != -1) {
        checkNavigation(currentDist);
    }
}

// ========================================================
// === RADAR SUAVIZADO
// ========================================================
int updateRadar() {
    if (millis() - lastScanTime > SCAN_INTERVAL) {
        lastScanTime = millis();

        currentServoPos += (SCAN_STEP * servoDirection);
        
        // Verifica limites e inverte
        if (currentServoPos >= SCAN_ANGLE_MAX) {
            currentServoPos = SCAN_ANGLE_MAX;
            servoDirection = -1;
        } else if (currentServoPos <= SCAN_ANGLE_MIN) {
            currentServoPos = SCAN_ANGLE_MIN;
            servoDirection = 1;
        }
        
        headServo.write(currentServoPos);

        // Leitura
        int cm = sonar.ping_cm();
        return (cm == 0) ? 999 : cm;
    }
    return -1;
}

// ========================================================
// === NAVEGAÇÃO
// ========================================================
void checkNavigation(int dist) {
    if (dist <= DIST_STOP) {
        stopMotors();
        Serial.println("!!! OBSTACULO !!!");
        
        headServo.write(SERVO_CENTER);
        delay(300); 
        
        // Tira-teima
        int check = sonar.ping_cm();
        if (check > 0 && check <= DIST_STOP + 5) {
             decideRoute(); 
        } else {
             wasStopped = true; 
        }
        lastScanTime = millis();
    }
    else if (dist <= DIST_WARN) {
        setSpeedAll((int)(130 * SPEED_MODIFIER));
        moveForward();
        wasStopped = false;
    }
    else {
        if (wasStopped) {
            kickStartForward();
        } else {
            setSpeedAll(SPEED_CRUISE);
            moveForward();
        }
    }
}

// ========================================================
// === MANOBRA
// ========================================================
void decideRoute() {
    Serial.println("--- DECIDINDO ---");
    
    // Recuo
    setSpeedAll(SPEED_KICK);
    moveBackward();
    delay(KICK_TIME + RE);
    stopMotors();
    delay(300);

    // Olha para a DIREITA primeiro (Angulo 10)
    headServo.write(10); delay(600);
    int distDir = getStableDistance();
    
    // Olha para a ESQUERDA depois (Angulo 170)
    headServo.write(170); delay(600);
    int distEsq = getStableDistance();

    // Centraliza
    headServo.write(90); delay(400);

    Serial.print("Dir: "); Serial.print(distDir);
    Serial.print(" | Esq: "); Serial.println(distEsq);

    if (distEsq >= distDir && distEsq > DIST_STOP) {
        smartTurnLeft();
    }
    else if (distDir > distEsq && distDir > DIST_STOP) {
        smartTurnRight();
    }
    
    stopMotors();
    delay(500);
    wasStopped = true;
}

// ========================================================
// === HELPERS
// ========================================================

void smartTurnLeft() { turnLeftTank(TURN_DELAY_90_DEG); }
void smartTurnRight() { turnRightTank(TURN_DELAY_90_DEG); }

void kickStartForward() {
    setSpeedAll(SPEED_KICK);
    moveForward();
    delay(KICK_TIME);
    setSpeedAll(SPEED_CRUISE);
    wasStopped = false;
}

int getStableDistance() {
    int cm = sonar.ping_median(5) / US_ROUNDTRIP_CM;
    return (cm == 0) ? 999 : cm;
}

void setSpeedAll(int s) {
    motorEsqFrente.setSpeed(s); motorDirFrente.setSpeed(s);
    motorEsqTras.setSpeed(s); motorDirTras.setSpeed(s);
}

void stopMotors() {
    motorEsqFrente.run(RELEASE); motorDirFrente.run(RELEASE);
    motorEsqTras.run(RELEASE); motorDirTras.run(RELEASE);
    wasStopped = true;
}

void moveForward() {
    motorEsqFrente.run(BACKWARD); motorEsqTras.run(BACKWARD);
    motorDirFrente.run(BACKWARD); motorDirTras.run(BACKWARD);
}

void moveBackward() {
    motorEsqFrente.run(FORWARD); motorEsqTras.run(FORWARD);
    motorDirFrente.run(FORWARD); motorDirTras.run(FORWARD);
}

void turnLeftTank(int t) {
    setSpeedAll(SPEED_TURN);
    motorEsqFrente.run(FORWARD);  motorEsqTras.run(FORWARD);
    motorDirFrente.run(BACKWARD); motorDirTras.run(BACKWARD);
    delay(t);
}

void turnRightTank(int t) {
    setSpeedAll(SPEED_TURN);
    motorEsqFrente.run(BACKWARD); motorEsqTras.run(BACKWARD);
    motorDirFrente.run(FORWARD);  motorDirTras.run(FORWARD);
    delay(t);
}