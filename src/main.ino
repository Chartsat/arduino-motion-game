#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <Wire.h>
#include <Kalman.h>
#include <Fonts/greek7pt7b.h>


#define RESTRICT_PITCH

#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF

int* objectXCoords;
int* objectYCoords;

Kalman kalmanX;  // Create the Kalman instances
Kalman kalmanY;

MCUFRIEND_kbv tft;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle;  // Angle calculate using the gyro only
double compAngleX, compAngleY;  // Calculated angle using a complementary filter
double kalAngleX, kalAngleY;    // Calculated angle using a Kalman filter

int targetX;
int targetY;

float roll;
float pitch;

float posX;
float posY;
int ballRadius = 5;

int rectX = 40; //RectX and RectY give the point where the angle of board start
int rectY = 90; 
int rectWidth = 160; //rectWidth and rectHeight draws the board careful with the dimension
int rectHeight = 190;
int left, right, top, bottom;

int score = 0;
int current_stage = 0;
int score_threshold = 0;
int numObjects = 0;
int maxObjects = 20; // maximum number of objects per stage


int success_counter = 0;
unsigned long start_time;
int sec = 60;
int high_score; 

boolean start = false;
bool targetDisplayed = false; // initialize targetDisplayed to false

uint32_t timer;
uint8_t i2cData[14];  // Buffer for I2C data



void setup() {


  //EEPROM.write(9, 0);         // reset high score to 0
  if (EEPROM.read(9) == 255) {    // if EEPROM is still at default value 255
    EEPROM.write(9, 0);           // reset GYRO high score to 0
  }
  randomSeed(analogRead(0));
  Serial.begin(9600);
  Wire.begin();
  tft.reset();
  uint16_t id = tft.readID();
  tft.begin(id);
  tft.fillRect(0, 0, 240, 320, WHITE);
  tft.setFont(&greek7pt7b); 
  //tft.setRotation(0); // Set Rotation at 90 degress


  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll  = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  high_score = EEPROM.read(9);
  timer = micros();
  start_time = millis();

  objectXCoords = new int[maxObjects];
  objectYCoords = new int[maxObjects];
}

void loop() {
  if (start == false) {
    drawIntroGame();
    start = true;
  }
  getIMU();
  drawIMUgame();
  if (success_counter >= score_threshold) {
    stages();
  }
  updateScore(success_counter);
}

void getIMU() {

  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll  = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
}

void drawIMUgame() {

  getRectSides(rectX, rectY, rectWidth, rectHeight, left, right, top, bottom);
  tft.drawRect(rectX, rectY, rectWidth, rectHeight, RED);
  
    if (!targetDisplayed) {
    // generate random target coordinates and draw the target
    targetX = random(rectWidth - 10) + rectX + 5;
    targetY = random(rectHeight - 10) + rectY + 5;
    drawTarget(targetX, targetY);
    targetDisplayed = true; // set targetDisplayed to true
    
    // Set the initial position of the ball to be in the center of the rectangle
    int centerX = rectX + (rectWidth / 2);
    int centerY = rectY + (rectHeight / 2);
    posX = centerX;
    posY = centerY;
  }

  // Reset the previous position of the ball
  tft.fillRect(posX - 5, posY - 5, 11, 11, WHITE);
  
  // Update the position of the ball based on sensor data
  posX -= roll / 5.0; // speed 
  posY += pitch / 5.0;
  
  // Draw the ball
  tft.fillCircle(posX, posY, ballRadius, GREEN);
  
  // Check if the ball has reached the target
  if (posX > targetX - 5 && posX < targetX + 5 && posY > targetY - 5 && posY < targetY + 5) {
    success_counter++;
    // Move the target to a new position
    targetX = random(rectWidth - 10) + rectX + 5;
    targetY = random(rectHeight - 10) + rectY + 5;
    drawTarget(targetX, targetY);
  }

  checkBorderHit(posX, posY, ballRadius, left, right, top, bottom);
  checkCollisions(posX, posY, ballRadius, numObjects, objectXCoords, objectYCoords);
  checkTimeLimit(start_time, sec, success_counter, high_score);
 
}


void updateScore(int score) {
  tft.setTextColor(BLACK); 
  tft.setCursor(180, 10);
  tft.print("BAQMOS");
  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.setCursor(200, 20);
  tft.fillRect(200, 20, 50, 50, WHITE); 
  tft.print(score);
  tft.setFont(&greek7pt7b);
  tft.setTextSize(NULL);
}

void drawIntroGame() {
  tft.setTextColor(BLACK);
  tft.setCursor(30, 120);
  tft.print("CARILAOS TSATSARHS 3313");
  tft.setTextColor(BLACK);
  tft.setCursor(70, 140);
  tft.print("H MPALA");
  tft.setTextColor(BLACK);
  tft.setCursor(75, 160);
  tft.print("REKOR: ");
  tft.print(high_score);
  delay(2000);
  tft.fillRect(0, 0, 240, 320, WHITE);
}

void getRectSides(int rectX, int rectY, int rectWidth, int rectHeight,
                  int& left, int& right, int& top, int& bottom) {
  left = rectX;
  right = rectX + rectWidth;
  top = rectY;
  bottom = rectY + rectHeight;
}

void drawTarget(int targetX, int targetY) {
  int rectX = targetX - 5;
  int rectY = targetY - 5;
  int rectWidth = 10;
  int rectHeight = 10;
  int left, right, top, bottom;
  getRectSides(rectX, rectY, rectWidth, rectHeight, left, right, top, bottom);
  tft.drawRect(rectX, rectY, rectWidth, rectHeight, BLACK);
}

void addObject() {
  if (numObjects < maxObjects) { // if the maximum number of objects for the stage has not been reached
    // Generate random object coordinates within the rectangle
    int left, right, top, bottom;
    getRectSides(rectX, rectY, rectWidth, rectHeight, left, right, top, bottom);
    int objectX = random(rectWidth - 10) + rectX + 5;
    int objectY = random(rectHeight - 10) + rectY + 5;

    // Draw the object
    tft.fillCircle(objectX, objectY, 5, BLUE);

    // Add the object's coordinates to the arrays
    objectXCoords[numObjects] = objectX;
    objectYCoords[numObjects] = objectY;

    numObjects++;
  }
}

void stages() {
  while (success_counter >= score_threshold) {
    // Increase the current stage and update the score threshold for the next stage
    current_stage++;
    score_threshold += 3;

    // Add objects for the new stage
    for (int i = 1; i < current_stage; i++) {
      addObject();
    }

    //Calculate the number of objects to add for this stage
    // int objectsToAdd = current_stage / 2;  add fewer objects in each stage

    // Update the stage display
    //tft.setCursor(80, 290);
    //tft.setTextColor(BLACK);
    //tft.print("EPIPEDO ");
    //tft.print(current_stage); 
    //tft.setCursor(80, 290);
  
  }
}

void checkCollisions(int posX, int posY, int radius, int numObjects, int* objectXCoords, int* objectYCoords) {
  bool collisionDetected = false;

  for (int i = 0; i < numObjects; i++) {
    int dx = posX - objectXCoords[i];
    int dy = posY - objectYCoords[i];
    int distance = sqrt(dx * dx + dy * dy);
    if (distance <= radius + 5) { // Check for collision with object
      tft.fillCircle(objectXCoords[i], objectYCoords[i], 5, BLUE);
      objectXCoords[i] = -10;
      objectYCoords[i] = -10;
      collisionDetected = true;
      break;  // stop checking further if a collision is detected
    }
  }
  
  if(collisionDetected) {
    tft.setTextColor(RED);
    tft.setCursor(40, 40);
    tft.print("CTUPHSES ANTIKEIMENO");
    tft.setCursor(60, 55);
    tft.print ("O BAQMOS SOU ");
    tft.print(success_counter);
    tft.setCursor(0, 290);
    tft.print ("PATA TO KOUMPI GIA NEO PAICNIDI");

     if (success_counter > high_score) {
      high_score = success_counter;
      EEPROM.write(9, high_score);
      tft.fillRect(0, 0, 240, 320, BLACK);
      tft.setTextColor(GREEN);
      tft.setCursor(80, 150);
      tft.print("NEOS REKOR ");
      tft.print(high_score);
      tft.setCursor(80, 170);
      tft.print("SUNCARHTHRIA");
      tft.setCursor(0, 290);
     tft.print ("PATA TO KOUMPI GIA NEO PAICNIDI");  
    }
        exit(0);
    }
}

void checkBorderHit(int posX, int posY, int ballRadius, int left, int right, int top, int bottom) {
  if (posX - ballRadius < left || posX + ballRadius > right || posY - ballRadius < top || posY + ballRadius > bottom) {
    // Ball hit the border
    tft.setTextColor(RED);
    tft.setCursor(70, 40);
    tft.print("EKTOS ORIWN");
    tft.setCursor(60, 55);
    tft.print ("O BAQMOS SOU ");
    tft.print(success_counter);
    tft.setCursor(0, 290);
    tft.print ("PATA TO KOUMPI GIA NEO PAICNIDI");

     if (success_counter > high_score) {
      high_score = success_counter;
      EEPROM.write(9, high_score);
      tft.fillRect(0, 0, 240, 320, BLACK);
      tft.setTextColor(GREEN);
      tft.setCursor(80, 150);
      tft.print("NEOS REKOR ");
      tft.print(high_score);
      tft.setCursor(80, 170);
      tft.print("SUNCARHTHRIA");
      tft.setCursor(0, 290);
      tft.print ("PATA TO KOUMPI GIA NEO PAICNIDI");
    }
    exit(0);
  }
}

void checkTimeLimit(unsigned long start_time, int sec, int success_counter, int& high_score) {

tft.setFont(NULL);
tft.setTextSize(2);
tft.setCursor(20, 20);
tft.fillRect(20, 20, 50, 50, WHITE);
tft.print(sec - (millis() - start_time) / 1000.0, 0);
tft.setTextSize(NULL);
tft.setFont(&greek7pt7b);
tft.setCursor(0, 10);
tft.print("DEUTEROLEPTA");

  if ((millis() - start_time) / 1000.0 >= sec) { 
    tft.setTextColor(RED);
    tft.setCursor(70, 40);
    tft.print("LHXH XRONOU");  
    tft.setCursor(60, 55);
    tft.print ("O BAQMOS SOU ");
    tft.print(success_counter);
    tft.setCursor(0, 290);
    tft.print ("PATA TO KOUMPI GIA NEO PAICNIDI");

  if (success_counter > high_score) { 
      high_score = success_counter;
      EEPROM.write(9, high_score);
      tft.fillRect(0, 0, 240, 320, BLACK);
      tft.setTextColor(GREEN);
      tft.setCursor(80, 150);
      tft.print("NEO REKOR ");
      tft.print(high_score);
      tft.setCursor(80, 170);
      tft.print("SUNCARHTHRIA");
      tft.setCursor(0, 290);
      tft.print ("PATA TO KOUMPI GIA NEO PAICNIDI");
}
    exit(0);

}
}

