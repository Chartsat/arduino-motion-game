# The Ball - A Gyroscope-Controlled Game

## Description
"The Ball" is an interactive game for an Arduino-based device with an attached LCD display and an MPU-6050 gyroscope/accelerometer. The player controls a green ball on the screen by tilting the device, aiming to guide it to a black target. The game features multiple stages, increasing in difficulty as the player progresses by adding more obstacles (blue circles). The objective is to achieve the highest score by reaching as many targets as possible within a 60-second time limit. The game tracks the high score using the Arduino's EEPROM.

The game is written in C++ for the Arduino platform and uses several libraries to handle the display, IMU communication, and a Kalman filter for stable sensor readings. The on-screen text is in Greek.



## Features
* **Tilt-based Control:** Use an MPU-6050 sensor to control the ball's movement.
* **Kalman Filter:** Implements a Kalman filter for smooth and accurate sensor data, reducing noise and drift.
* **Score Tracking:** Tracks the player's score in real-time.
* **High Score Functionality:** Saves the highest score to the EEPROM, so it's remembered even after the device is powered off.
* **Multi-Stage Difficulty:** Increases the number of obstacles as the player's score increases.
* **Game Over Conditions:** The game ends if the ball hits a border, an obstacle, or the 60-second time limit expires.
* **Intuitive UI:** Displays the score and a countdown timer on the screen.

## Libraries
This project requires the following libraries:
* `SPI.h`: Standard SPI library for communication.
* `EEPROM.h`: For storing the high score.
* `Adafruit_GFX.h`: Core graphics library for the display.
* `MCUFRIEND_kbv.h`: For controlling the specific LCD display.
* `Wire.h`: For I2C communication with the MPU-6050.
* `Kalman.h`: A custom or third-party library for the Kalman filter.
* `Fonts/greek7pt7b.h`: A custom font for displaying Greek characters.

You may need to download and install these libraries via the Arduino IDE's Library Manager or from their respective GitHub repositories.

## Hardware Requirements
* Arduino board (e.g., Arduino Uno, Mega)
* TFT LCD display with a compatible MCUFRIEND shield.
* MPU-6050 Gyroscope/Accelerometer sensor.
* Jumper wires for connections.

## How to Install and Run
1.  **Install the Arduino IDE:** Download and install the Arduino IDE from the official website.
2.  **Install Libraries:** Open the Arduino IDE, go to `Sketch` -> `Include Library` -> `Manage Libraries...`. Search for and install the required libraries listed above. The `Kalman.h` and `greek7pt7b.h` libraries may need to be added manually by placing them in your Arduino sketch folder.
3.  **Connect Hardware:**
    * Attach the MCUFRIEND LCD shield to the Arduino board.
    * Connect the MPU-6050 sensor to the Arduino using I2C:
        * `SCL` pin on MPU-6050 to `A5` on Arduino.
        * `SDA` pin on MPU-6050 to `A4` on Arduino.
        * `VCC` to `5V`
        * `GND` to `GND`
4.  **Upload the Code:**
    * Copy and paste the provided code into a new sketch in the Arduino IDE.
    * Select the correct board and port from the `Tools` menu.
    * Click the `Upload` button to transfer the code to your Arduino.

## Gameplay
1.  The game starts with an intro screen showing the developer's name, the game's title, and the current high score.
2.  After a short delay, a green ball appears in the center of the board, and a black target is drawn at a random location.
3.  Tilt the device to move the green ball. Guide the ball to the target to score a point.
4.  Each time you score, a new target appears. The game progresses to the next stage every three successful hits, adding a blue obstacle to the board.
5.  Avoid hitting the blue obstacles or the red borders of the board.
6.  The game ends if you hit an obstacle, a border, or the 60-second timer runs out.
7.  If you beat the high score, a "NEOS REKOR" (New Record) message is displayed.
8.  To start a new game, you'll need to press a button (the code mentions a button, but it's not explicitly defined; a reset button on the Arduino will work).

## Code Structure
* **`setup()`:** Initializes the serial monitor, I2C communication with the MPU-6050, and the TFT display. It also reads the high score from EEPROM and performs the initial sensor calibration.
* **`loop()`:** The main game loop that continuously reads IMU data, updates the ball's position, checks for collisions and time limits, and updates the score.
* **`getIMU()`:** Reads raw data from the MPU-6050 and calculates the roll and pitch angles using a Kalman filter for stability.
* **`drawIMUgame()`:** Handles the game's graphics, including drawing the board, the ball, and the target. It also updates the ball's position based on the calculated angles.
* **`updateScore()`:** Displays the current score on the screen.
* **`drawIntroGame()`:** Displays the initial game splash screen.
* **`getRectSides()`:** A helper function to get the coordinates of a rectangle's sides.
* **`drawTarget()`:** Draws the target square on the screen.
* **`addObject()`:** Adds a new obstacle (blue circle) to the game board.
* **`stages()`:** Manages the game's stage progression, adding obstacles as the score increases.
* **`checkCollisions()`:** Detects if the ball hits a blue obstacle and ends the game if it does. It also updates the high score if a new one is achieved.
* **`checkBorderHit()`:** Detects if the ball hits the red border of the game board, ending the game.
* **`checkTimeLimit()`:** Manages the 60-second countdown and ends the game when the time expires.
