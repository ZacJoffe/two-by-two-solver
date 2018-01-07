/*
[2017/2018] Zac Joffe, Mark Robertson, Sam Stephenson

Brief description of code:

This code is intended to be used with a 2x2 rubik's cube. The cube being used
for this project is a white colour plastic square V-cube 2.

However, the cube had to be modified in two ways for this project. First off,
the black sticker with the V-cube logo was coloured in with a black Sharpie so
that the sensor does not confuse a black input for something else.

The colour sensor is not able to measure orange, so the orange stickers had to
be peeled off.

This code uses an algorithm created by the GitHub user wbernoudy. The project
is licensed as completely free to use, and can be found at the link below:
https://github.com/wbernoudy/pocket-cube-solver
*/


#include "EV3_FileIO.c"

const int SIDES = 6; // six sides per cube
const int ROWS = 2; // used for 2x2 2D arrays
const int COLS = 2;
const int MAX_MOVES = 14; // max possible amount of moves it could
						  //take to solve the 2x2

// used for the sensors, made global to use in functions as arrays cannot
// be passed as parameters in Robotc
int faceTop[ROWS][COLS];
int faceBack[ROWS][COLS];
int faceBottom[ROWS][COLS];
int faceFront[ROWS][COLS];
int faceRight[ROWS][COLS];
int faceLeft[ROWS][COLS];

// used for output to the external Java program
char faceTopChar[ROWS][COLS];
char faceBackChar[ROWS][COLS];
char faceBottomChar[ROWS][COLS];
char faceFrontChar[ROWS][COLS];
char faceRightChar[ROWS][COLS];
char faceLeftChar[ROWS][COLS];

string solution[MAX_MOVES];

void PID(int target, int motorPort)
{
	float kp = 0.8; // PID constants
	float ki = 0.8;
	float kd = 0.8;

	float kpValue = 0; // values once adjusted
	float kiValue = 0;
	float kdValue = 0;

	int error_last = 0;
	int error_diff = 0;
	int error_sum = 0;
	int maxTime = 2000;

	clearTimer(T1);
	nMotorEncoder[motorPort] = 0;

	int motorPower;

	while (time1[T1] < maxTime)
	{
		int currEncoder;

		if (target > 0)
			currEncoder = abs(nMotorEncoder[motorPort]);
		else
			currEncoder = nMotorEncoder[motorPort];

		int error = target - currEncoder;

		error_diff = error - error_last;
		error_last = error;

		kpValue = kp * error;
		kiValue = ki * error_sum;
		kdValue = kd * error_diff;

		// motorPower cap
		if (kpValue + kiValue + kdValue > 100)
			motorPower = 100;
		else if (kpValue + kiValue + kdValue < -100)
			motorPower = -100;
		else
			motorPower = (kpValue + kiValue + kdValue);

		motor[motorPort] = motorPower;

		delay(50);
	}
} 

void flipCube(int numFlips) // flips cube in the tray
{
	nMotorEncoder[motorA] = 0;

	motor[motorA] = 25;

	while (nMotorEncoder[motorA] < 85)
	{}

	motor[motorA] = 0;
	wait1Msec(200);

	for (int runs = 1; runs <= numFlips; runs++)
	{
		nMotorEncoder[motorA] = 0;

		motor[motorA] = 20;

		while (nMotorEncoder[motorA] < 75)
		{}

		motor[motorA] = 0;
		wait1Msec(200);

		nMotorEncoder[motorA] = 0;

		motor[motorA] = -20;

		while (nMotorEncoder[motorA] > -75)
		{}

		motor[motorA] = 0;
		wait1Msec(400);
	}

	nMotorEncoder[motorA] = 0;

	clearTimer(T2);

	motor[motorA] = -25;

	while (nMotorEncoder[motorA] > -85 && time1[T2] <= 2000)
	{}

	motor[motorA] = 0;
} 

void turnCubeCW(int numRotations) // turns whole cube clockwise by numRotations
								  //quarter turns
{
	PID(numRotations * 90 * 3, motorC);
	motor[motorC] = 0;
} 

void turnCubeCCW(int numRotations) // turns whole cube counterclockwise by
								   // numRotations quarter turns
{
	PID(-numRotations * 90 * 3, motorC);
	motor[motorC] = 0;
} 

void turnCW(int numRotations) // turns bottom face clockwise by numRotations
							  // quarter turns
{
	nMotorEncoder[motorA] = 0;

	motor[motorA] = 20;

	while (nMotorEncoder[motorA] < 85)
	{}

	motor[motorA] = 0;
	wait1Msec(200);

	PID(numRotations * 90 * 3 + 65, motorC); // does one turn
	PID(-65, motorC);
	motor[motorC] = 0;

	nMotorEncoder[motorA] = 0;

	clearTimer(T2);

	motor[motorA] = -20;

	while (nMotorEncoder[motorA] > -85 && time1[T2] <= 2000)
	{}

	motor[motorA] = 0;
} 

void turnCCW(int numRotations) // turns bottom face counterclockwise
							   // by numRotations quarter turns
{
	nMotorEncoder[motorA] = 0;

	motor[motorA] = 20;

	while (nMotorEncoder[motorA] < 85)
	{}

	motor[motorA] = 0;
	wait1Msec(200);

	PID(-numRotations * 90 * 4.5 + 65, motorC); // does one turn
	PID(65, motorC);
	motor[motorC] = 0;

	nMotorEncoder[motorA] = 0;

	clearTimer(T2);

	motor[motorA] = -20;

	while (nMotorEncoder[motorA] > -85 && time1[T2] <= 2000)
	{}

	motor[motorA] = 0;
} 

void moveColourForward(bool isMovedForward) // moves colour sensor
											// forward or back
{
	nMotorEncoder[motorB] = 0;

	if (isMovedForward)
	{
		motor[motorB] = -25; // negative is forward
		while (nMotorEncoder[motorB] > -615)
		{}
	}
	else
	{
		motor[motorB] = 25;
		while (nMotorEncoder[motorB] < 615)
		{}
	}

	motor[motorB] = 0;
} 

void inspectCube() // places colour values into the six different int 2D arrays
{
	int numFlips = 1, numTurns = 1;

	for (int face = 0; face < SIDES; face++)
	{
		moveColourForward(true);
		turnCubeCCW(numTurns * 2);
		numTurns = 1;

		for (int row = 0; row < ROWS; row++)
		{
			for (int col = 0; col < COLS; col++)
			{
				if (face == 0) // top
				{
					if (row == 0)
					{
						wait1Msec(100);
						faceTop[row][col] = SensorValue[S1];
						wait1Msec(100);
						turnCubeCCW(numTurns);
					}
					else
					{
						turnCubeCCW(numTurns);
						wait1Msec(100);
						faceTop[row][col] = SensorValue[S1];
						wait1Msec(100);
						numTurns *= -1;
					}
				}
				else if (face == 1) // back
				{
					if (row == 0)
					{
						wait1Msec(100);
						faceBack[row][col] = SensorValue[S1];
						wait1Msec(100);
						turnCubeCCW(numTurns);
					}
					else
					{
						turnCubeCCW(numTurns);
						wait1Msec(100);
						faceBack[row][col] = SensorValue[S1];
						wait1Msec(100);
						numTurns *= -1;
					}
				}
				else if (face == 2) // bottom
				{
					if (row == 0)
					{
						wait1Msec(100);
						faceBottom[row][col] = SensorValue[S1];
						wait1Msec(100);
						turnCubeCCW(numTurns);
					}
					else
					{
						turnCubeCCW(numTurns);
						wait1Msec(100);
						faceBottom[row][col] = SensorValue[S1];
						wait1Msec(100);
						numTurns *= -1;
					}
				}
				else if (face == 3) // front
				{
					if (row == 0)
					{
						faceFront[row][col] = SensorValue[S1];
						turnCubeCCW(numTurns);
					}
					else
					{
						turnCubeCCW(numTurns);
						wait1Msec(100);
						faceFront[row][col] = SensorValue[S1];
						wait1Msec(100);
						numTurns *= -1;
					}
				}
				else if (face == 4) // right
				{
					if (row == 0)
					{
						wait1Msec(100);
						faceRight[row][col] = SensorValue[S1];
						wait1Msec(100);
						turnCubeCCW(numTurns);
					}
					else
					{
						turnCubeCCW(numTurns);
						wait1Msec(100);
						faceRight[row][col] = SensorValue[S1];
						wait1Msec(100);
						numTurns *= -1;
					}
				}
				else if (face == 5) // left
				{
					if (row == 0)
					{
						wait1Msec(100);
						faceLeft[row][col] = SensorValue[S1];
						wait1Msec(100);
						turnCubeCCW(numTurns);
					}
					else
					{
						turnCubeCCW(numTurns);
						wait1Msec(100);
						faceLeft[row][col] = SensorValue[S1];
						wait1Msec(100);
						numTurns *= -1;
					}
				}
			}
		}

		if (face <= 2)
		{
			moveColourForward(false);
			flipCube(numFlips);
		}
		else if (face == 3)
		{
			moveColourForward(false);
			flipCube(numFlips * 2);
			turnCubeCCW(numTurns);
			flipCube(numFlips);
			turnCubeCW(numTurns);
		}
		else if (face == 4)
		{
			moveColourForward(false);
			flipCube(numFlips * 2);
			turnCubeCCW(numTurns * 2);
		}
		else // restores cube to original position as placed
		{
			moveColourForward(false);
			turnCubeCW(numTurns);
			flipCube(numFlips);
			turnCubeCW(numTurns);
			flipCube(numFlips);
		}
	}
} 

void changeArrayToChar() // places the values of the int 2D arrays in
						 // the char 2D arrays for the Java program
{
	for (int face = 0; face < SIDES; face++)
	{
		for (int row = 0; row < ROWS; row++)
		{
			for (int col = 0; col < COLS; col++)
			{
				if (face == 0) // top
				{
					if (faceTop[row][col] == 1)
						faceTopChar[row][col] = 'w';
					else if (faceTop[row][col] == 2)
						faceTopChar[row][col] = 'b';
					else if (faceTop[row][col] == 3)
						faceTopChar[row][col] = 'g';
					else if (faceTop[row][col] == 4)
						faceTopChar[row][col] = 'y';
					else if (faceTop[row][col] == 5)
						faceTopChar[row][col] = 'r';
					else
						faceTopChar[row][col] = 'o';
				}
				else if (face == 1) // back
				{
					if (faceBack[row][col] == 1)
						faceBackChar[row][col] = 'w';
					else if (faceBack[row][col] == 2)
						faceBackChar[row][col] = 'b';
					else if (faceBack[row][col] == 3)
						faceBackChar[row][col] = 'g';
					else if (faceBack[row][col] == 4)
						faceBackChar[row][col] = 'y';
					else if (faceBack[row][col] == 5)
						faceBackChar[row][col] = 'r';
					else
						faceBackChar[row][col] = 'o';
				}
				else if (face == 2) // bottom
				{
					if (faceBottom[row][col] == 1)
						faceBottomChar[row][col] = 'w';
					else if (faceBottom[row][col] == 2)
						faceBottomChar[row][col] = 'b';
					else if (faceBottom[row][col] == 3)
						faceBottomChar[row][col] = 'g';
					else if (faceBottom[row][col] == 4)
						faceBottomChar[row][col] = 'y';
					else if (faceBottom[row][col] == 5)
						faceBottomChar[row][col] = 'r';
					else
						faceBottomChar[row][col] = 'o';
				}
				else if (face == 3) // front
				{
					if (faceFront[row][col] == 1)
						faceFrontChar[row][col] = 'w';
					else if (faceFront[row][col] == 2)
						faceFrontChar[row][col] = 'b';
					else if (faceFront[row][col] == 3)
						faceFrontChar[row][col] = 'g';
					else if (faceFront[row][col] == 4)
						faceFrontChar[row][col] = 'y';
					else if (faceFront[row][col] == 5)
						faceFrontChar[row][col] = 'r';
					else
						faceFrontChar[row][col] = 'o';
				}
				else if (face == 4) // right
				{
					if (faceRight[row][col] == 1)
						faceRightChar[row][col] = 'w';
					else if (faceRight[row][col] == 2)
						faceRightChar[row][col] = 'b';
					else if (faceRight[row][col] == 3)
						faceRightChar[row][col] = 'g';
					else if (faceRight[row][col] == 4)
						faceRightChar[row][col] = 'y';
					else if (faceRight[row][col] == 5)
						faceRightChar[row][col] = 'r';
					else
						faceRightChar[row][col] = 'o';
				}
				else // left
				{
					if (faceLeft[row][col] == 1)
						faceLeftChar[row][col] = 'w';
					else if (faceLeft[row][col] == 2)
						faceLeftChar[row][col] = 'b';
					else if (faceLeft[row][col] == 3)
						faceLeftChar[row][col] = 'g';
					else if (faceLeft[row][col] == 4)
						faceLeftChar[row][col] = 'y';
					else if (faceLeft[row][col] == 5)
						faceLeftChar[row][col] = 'r';
					else
						faceLeftChar[row][col] = 'o';
				}
			}
		}
	}
} 

void output(TFileHandle fout) // outputs scarmbled state of the cube
							  // to a file to be read in by the Java program
{
	for (int face = 0; face < SIDES; face++)
	{
		for (int row = 0; row < ROWS; row++)
		{
			for (int col = 0; col < COLS; col++)
			{
				if (face == 0)
				{
					writeCharPC(fout, faceTopChar[row][col]);
					writeEndlPC(fout);
				}
				else if (face == 1)
				{
					writeCharPC(fout, faceBackChar[row][col]);
					writeEndlPC(fout);
				}
				else if (face == 2)
				{
					writeCharPC(fout, faceBottomChar[row][col]);
					writeEndlPC(fout);
				}
				else if (face == 3)
				{
					writeCharPC(fout, faceFrontChar[row][col]);
					writeEndlPC(fout);
				}
				else if (face == 4)
				{
					writeCharPC(fout, faceRightChar[row][col]);
					writeEndlPC(fout);
				}
				else
				{
					writeCharPC(fout, faceLeftChar[row][col]);
					writeEndlPC(fout);
				}
			}
		}

		writeEndlPC(fout);
	}
} 

int solve() // solves cube and returns the num moves that it takes to solve
{
	bool flag = true;
	int count = 0;

	for (int index = 0; index < MAX_MOVES && flag; index++)
	{
		if (solution[index] == "F")
		{
			turnCubeCCW(2);
			flipCube(1);
			turnCCW(1);
			turnCubeCCW(2);
			flipCube(1);

			wait1Msec(1500);

			count++;
		}
		else if (solution[index] == "R")
		{
			turnCubeCCW(1);
			flipCube(1);
			turnCCW(1);
			turnCubeCCW(2);
			flipCube(1);
			turnCubeCCW(1);

			wait1Msec(1500);

			count++;
		}
		else if (solution[index] == "T")
		{
			flipCube(2);
			turnCCW(1);
			flipCube(2);

			wait1Msec(1500);

			count++;
		}
		else if (solution[index] == "F'")
		{
			turnCubeCCW(2);
			flipCube(1);
			turnCW(1);
			turnCubeCW(2);
			flipCube(1);

			wait1Msec(1500);

			count++;
		}
		else if (solution[index] == "R'")
		{
			turnCubeCCW(1);
			flipCube(1);
			turnCW(1);
			turnCubeCCW(2);
			flipCube(1);
			turnCubeCCW(1);

			wait1Msec(1500);

			count++;
		}
		else if (solution[index] == "T'")
		{
			flipCube(2);
			turnCW(1);
			flipCube(2);

			wait1Msec(1500);

			count++;
		}
		else
			flag = false;
	}

	return count;
} 

task main()
{
	// colour is in s1, ultrasonic s2
	// flipper motor is A, colour is B, tray is C

	SensorType[S1] = sensorEV3_Color;
	wait1Msec(50);
	SensorMode[S1] = modeEV3Color_Color;
	wait1Msec(50);

	SensorType[S2] = sensorEV3_Ultrasonic;

	while (SensorValue[S2] >= 12) // waits for cube to be placed in tray
	{}

	displayString(0, "Press enter button to start.");

	while (!getButtonPress(buttonEnter))
	{}

	while (getButtonPress(buttonEnter))
	{}

	eraseDisplay();

	inspectCube();

	TFileHandle fout; // setup file to be outputed
	bool fileOutputOkay = openWritePC(fout, "scramble.txt");

	changeArrayToChar();

	output(fout);

	while (!getButtonPress(buttonEnter)) // wait for solution file to be uploaded to the EV3
	{
		motor[motorC] = 0;
		bFloatDuringInactiveMotorPWM = true; // allows tray to coast to realign
	}

	while (getButtonPress(buttonEnter))
	{}

	TFileHandle fin; // setup file to be inputed
	bool fileInputOkay = openReadPC(fin, "solution.txt");

	for (int index = 0; index < MAX_MOVES; index++)
	{
		readTextPC(fin, solution[index]);
	}

	int numMoves = solve();

	displayString(0, "It took %d moves to solve!", numMoves);
	displayString(2, "Press any button to end.");

	while (!getButtonPress(buttonAny))
	{}

	while (getButtonPress(buttonAny))
	{}

	closeFilePC(fout);
	closeFilePC(fin);
} 