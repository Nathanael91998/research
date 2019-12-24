#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include <fstream>
#include <complex>

using namespace std;

class Position
{
	protected:
		int xPosition;	
		int yPosition;
	public:
		Position();
		Position(int x, int y);
		void display();
		void setXPosition(int x);
		void setYPosition(int y);
		int getXPosition();
		int getYPosition();
		vector<int> calculateDistanceErrorX(vector<Position> reckoned, vector<Position> actual, int noDrones);
		vector<int> calculateDistanceErrorY(vector<Position> reckoned, vector<Position> actual, int noDrones);
};

Position::Position() 
{
	xPosition = NULL;
	yPosition = NULL;
}


Position::Position(int x, int y)
{
	xPosition = x;
	yPosition = y;
}



void Position::setXPosition(int x)
{
	xPosition = x;
}

void Position::setYPosition(int y) 
{
	yPosition = y;
}

int Position::getXPosition() 
{
	return xPosition;
}

int Position::getYPosition()
{
	return yPosition;
}

void Position::display()
{
	cout << "(" << xPosition << ", " << yPosition << ")" << endl;
}

vector<int> Position::calculateDistanceErrorX(vector<Position> actual, vector<Position> reckoned, int noDrones)
{
	vector<int> *returnVector = new vector<int>;
	int difference = 0;

	for (int i = 0; i < noDrones; i++) 
	{
		difference = actual.at(i).getXPosition() - reckoned.at(i).getXPosition();
		(*returnVector).push_back(difference);
	}

	return *returnVector;
}

vector<int> Position::calculateDistanceErrorY(vector<Position> actual, vector<Position> reckoned, int noDrones)
{
	vector<int> *returnVector = new vector<int>;
	int difference = 0;

	for (int i = 0; i < noDrones; i++)
	{
		difference = actual.at(i).getYPosition() - reckoned.at(i).getYPosition();
		(*returnVector).push_back(difference);
	}

	return *returnVector;
}



class Drone
{
	protected:
		int noCells;
		int relaxationParameter;
		int deadReckoningErrorsX;
		int deadReckoningErrorsY;
		double constantSpeed;
		double theta;
		vector<int> cellsCrossedVector;
		vector<double> lengthInEachCellVector;
		vector<double> speedInEachCellVector;
		vector<double> xFlowInEachCellVector;
		vector<double> yFlowInEachCellVector;
		vector<double> lengthDividedBySpeedVector;
		vector<double> finalCalculatedTermsVector;
	public:
		Drone();
		Drone(int noCells);
		void updateXFlow(int i, Drone last);
		void updateYFlow(int i, Drone last);
		void calculateNextDroneXFlow(Drone previous);
		void calculateNextDroneYFlow(Drone previous);
		double updateRateOfChangeX(Drone first);
		double updateRateOfChangeY(Drone first);
		double calculateLiXTimesFlow(Drone previous);
		double calculateLiYTimesFlow(Drone previous);
		double calculateAbsoluteValue();
		void multiplyByTranspose(double multiplier);
		void setLij(double value, int index);
		void displayLij(int index);
		void setDeadReckoningErrorX(int error);
		void setDeadReckoningErrorY(int error);
		void setSpeed(double speed, int index);
		void setLengthDividedBySpeed(double length, double speed, int index);
		void setTheta(double t);
		void updateLij(Drone lastDrone);
};	

Drone::Drone()
{
	noCells = NULL;
	relaxationParameter = NULL;
	deadReckoningErrorsX = NULL;
	deadReckoningErrorsY = NULL;
}

Drone::Drone(int number):lengthInEachCellVector(number),speedInEachCellVector(number),cellsCrossedVector(number),xFlowInEachCellVector(number),yFlowInEachCellVector(number),lengthDividedBySpeedVector(number),finalCalculatedTermsVector(number)
{
	relaxationParameter = 1;
	noCells = number;
	theta = 0;
	for (int i = 0; i < number; i++)
	{
		lengthInEachCellVector.at(i) = 0;
		speedInEachCellVector.at(i) = 0;
		cellsCrossedVector.at(i) = 0;
		xFlowInEachCellVector.at(i) = 0;
		yFlowInEachCellVector.at(i) = 0;
		lengthDividedBySpeedVector.at(i) = 0;
		finalCalculatedTermsVector.at(i) = 0;
	}
	deadReckoningErrorsX = 0;
	deadReckoningErrorsY = 0;
}


void Drone::updateXFlow(int i, Drone last)
{
	for (int j = 0; j < i; j++)
	{
		(*this).xFlowInEachCellVector[j] = last.xFlowInEachCellVector[j];
	}
}

void Drone::updateYFlow(int i, Drone last)
{
	for (int j = 0; j < i; j++)
	{
		(*this).yFlowInEachCellVector[j] = last.yFlowInEachCellVector[j];
	}
}

double Drone::updateRateOfChangeX(Drone first)
{
	double totalValue = 0;
	double subtractedValue;
	for (int i = 0; i < noCells; i++)
	{
		subtractedValue = ((*this).xFlowInEachCellVector[i] - first.xFlowInEachCellVector[i]);
		totalValue = pow(subtractedValue, 2) + totalValue;
	}
	double returnValue = sqrt(totalValue);
	return returnValue;
}

double Drone::updateRateOfChangeY(Drone first)
{
	double totalValue = 0;
	double subtractedValue;
	for (int i = 0; i < noCells; i++)
	{
		subtractedValue = ((*this).yFlowInEachCellVector[i] - first.yFlowInEachCellVector[i]);
		totalValue = pow(subtractedValue, 2) + totalValue;
	}
	double returnValue = sqrt(totalValue);
	return returnValue;
}

void Drone::calculateNextDroneXFlow(Drone previous)
{
	double subtractingTerm = (*this).calculateLiXTimesFlow(previous);
	cout << "SubtractingTerm: " << subtractingTerm << endl;
	double dxi = (*this).deadReckoningErrorsX;
	cout << "dxi: " << dxi << endl;
	double topTerm = dxi - subtractingTerm;
	cout << "topTerm: " << topTerm << endl;
	double bottomTerm = (*this).calculateAbsoluteValue();
	cout << "bottomTerm: " << bottomTerm << endl;

	double wholeTerm = topTerm / bottomTerm;
	cout << "wholeTerm: " << wholeTerm << endl;
	double newWholeTerm = wholeTerm * previous.relaxationParameter;
	cout << "newWholeTerm: " << newWholeTerm << endl;

	(*this).multiplyByTranspose(newWholeTerm);

	for (int i = 0; i < noCells; i++)
	{
		(*this).xFlowInEachCellVector[i] = previous.xFlowInEachCellVector[i] + previous.finalCalculatedTermsVector[i];
	}
}

void Drone::calculateNextDroneYFlow(Drone previous)
{
	double subtractingTerm = (*this).calculateLiYTimesFlow(previous);
	cout << "SubtractingTerm: " << subtractingTerm << endl;
	double dyi = (*this).deadReckoningErrorsY;
	cout << "dyi: " << dyi << endl;
	double topTerm = dyi - subtractingTerm;
	cout << "topTerm: " << topTerm << endl;
	double bottomTerm = (*this).calculateAbsoluteValue();
	cout << "bottomTerm: " << bottomTerm << endl;

	double wholeTerm = topTerm / bottomTerm;
	cout << "wholeTerm: " << wholeTerm << endl;
	double newWholeTerm = wholeTerm * previous.relaxationParameter;
	cout << "newWholeTerm: " << newWholeTerm << endl;

	(*this).multiplyByTranspose(newWholeTerm);

	for (int i = 0; i < noCells; i++)
	{
		(*this).yFlowInEachCellVector[i] = previous.yFlowInEachCellVector[i] + previous.finalCalculatedTermsVector[i];
	}
}

double Drone::calculateLiXTimesFlow(Drone previous)
{
	double returnValue = 0;
	for (int i = 0; i < noCells; i++)
	{
		returnValue = returnValue + ((*this).lengthDividedBySpeedVector[i] * previous.xFlowInEachCellVector[i]);
	}
	return returnValue;
}

double Drone::calculateLiYTimesFlow(Drone previous)
{
	double returnValue = 0;
	for (int i = 0; i < noCells; i++)
	{
		returnValue = returnValue + ((*this).lengthDividedBySpeedVector[i] * previous.yFlowInEachCellVector[i]);
	}
	return returnValue;
}

double Drone::calculateAbsoluteValue()
{
	double returnValue = 0;
	for (int i = 0; i < noCells; ++i)
	{
		returnValue = returnValue + pow((*this).lengthDividedBySpeedVector[i],2);
	}
	return returnValue;
}


void Drone::multiplyByTranspose(double mult)
{
	double multiplier = mult;
	for (int i = 0; i < noCells; i++)
	{
		finalCalculatedTermsVector[i] = multiplier * lengthDividedBySpeedVector[i];
	}
}


void Drone::setLij(double value, int index)
{
	lengthInEachCellVector[index] = value;
}


void Drone::displayLij(int i) 
{
	cout << "Cell " << i + 1 << ": ";
	printf("%lf", lengthInEachCellVector[i]);
	cout << ", length/speed: ";
	printf("%lf", lengthDividedBySpeedVector[i]);
	//printf("%lf\t%lf", lengthInEachCell[i], lengthInEachCellVector[i]);
	cout << endl;
}

void Drone::setDeadReckoningErrorX(int e)
{
	deadReckoningErrorsX = e;
}

void Drone::setDeadReckoningErrorY(int e)
{
	deadReckoningErrorsY = e;
}

void Drone::setSpeed(double s, int index)
{
	speedInEachCellVector[index] = s;
	constantSpeed = s;
}

void Drone::setLengthDividedBySpeed(double l, double s, int index)
{
	lengthDividedBySpeedVector[index] = (l / s);
}

void Drone::setTheta(double t)
{
	theta = t;
}

void Drone::updateLij(Drone lastDrone)
{
	double newSpeed = 0;

	for (int i = 0; i < noCells; i++)
	{
		newSpeed = sqrt(pow((constantSpeed * cos(theta) + lastDrone.xFlowInEachCellVector.at(i)),2) + pow((constantSpeed * sin(theta) + lastDrone.yFlowInEachCellVector.at(i)), 2));
		speedInEachCellVector.at(i) = newSpeed;
		lengthDividedBySpeedVector.at(i) = lengthInEachCellVector.at(i) / speedInEachCellVector.at(i);
	}
}







int main() 
{
	ifstream inFile;
	inFile.open("C:\\Users\\Nathanael\\Documents\\ResearchInput.txt");
	if (!inFile) 
	{
		cerr << "Unable to open file ResearchInput.txt";
		exit(1);
	}


	int noDrones, noCells, travelTimeSeconds, xPosition, yPosition;
	double speedMetersPerSecond;
	inFile >> noDrones >> noCells >> travelTimeSeconds >> speedMetersPerSecond;

	cout << "Number of drones: " << noDrones << endl;
	cout << "Number of cells: " << noCells << endl;
	cout << "Time the drones will travel in seconds: " << travelTimeSeconds << endl;
	cout << "Speed of the drones in meters per second: ";
	printf("%lf", speedMetersPerSecond);
	cout << endl << endl;


	vector<Position> *startingPositions = new vector<Position>;
	vector<Position> *reckonedPositions = new vector<Position>;
	vector<Position> *actualPositions = new vector<Position>;
	Position* temp = new Position();

	//The first set of (# of drones) coordinates in the .txt file are the starting positions for each drone
	for(int i = 0; i < noDrones; i++)
	{
		inFile >> xPosition >> yPosition;
		temp = new Position(xPosition, yPosition);
		(*startingPositions).push_back(*temp);
	}

	//The second set of (# of drones) coordinates in the .txt file are the reckoned positions for each drone
	for (int i = 0; i < noDrones; i++)
	{
		inFile >> xPosition >> yPosition;
		temp = new Position(xPosition, yPosition);
		(*reckonedPositions).push_back(*temp);
	}

	//The third set of (# of drones) coordinates in the .txt file are the actual ending positions for each drone
	for (int i = 0; i < noDrones; i++)
	{
		inFile >> xPosition >> yPosition;
		temp = new Position(xPosition, yPosition);
		(*actualPositions).push_back(*temp);
	}



	//Calculate theta values for each drone

	double oppositeY = 0;
	double adjacentX = 0;
	double oppositeDividedByAdjacent = 0;
	double thetaRadians = 0;
	vector<double> *thetaForEachDrone = new vector<double>;
	double pi = 3.1415926535897;
	double thetaDegrees = 0;

	for (int i = 0; i < noDrones; i++)
	{
		oppositeY = (*actualPositions).at(i).getYPosition() - (*startingPositions).at(i).getYPosition();
		adjacentX = (*actualPositions).at(i).getXPosition() - (*startingPositions).at(i).getXPosition();
		oppositeDividedByAdjacent = oppositeY / adjacentX;
		thetaRadians = atan(oppositeDividedByAdjacent);
		thetaDegrees = (thetaRadians * 180) / pi;
		(*thetaForEachDrone).push_back(thetaDegrees);
	}






	//Display the coordinates for each drone to make sure that the data was correctly inputted
	cout << "Starting coordinates of each drone" << endl;
	for (int i = 0; i < noDrones; i++)
	{
		cout << "Drone " << i + 1 << ": ";
		(*startingPositions).at(i).display();
	}
	cout << endl;

	cout << "Reckoned coordinates of each drone" << endl;
	for (int i = 0; i < noDrones; i++)
	{
		cout << "Drone " << i + 1 << ": ";
		(*reckonedPositions).at(i).display();
	}
	cout << endl;

	cout << "Actual final coordinates of each drone" << endl;
	for (int i = 0; i < noDrones; i++)
	{
		cout << "Drone " << i + 1 << ": ";
		(*actualPositions).at(i).display();
	}
	cout << endl;

	//Now we formally begin the algorithm by computing di

	//Compute the distance error between the GPS and dead-reckoned surfacing positions
	vector<int> *distanceErrorX = new vector<int>;
	vector<int> *distanceErrorY = new vector<int>;

	
	(*distanceErrorX) = (*temp).calculateDistanceErrorX((*actualPositions), (*reckonedPositions), noDrones);
	(*distanceErrorY) = (*temp).calculateDistanceErrorY((*actualPositions), (*reckonedPositions), noDrones);


	//Display the distance error to confirm they were correctly calculated
	cout << "Drone distance error X direction: ";
	for (auto it = (*distanceErrorX).begin(); it != (*distanceErrorX).end(); ++it)
	{
		cout << ' ' << *it;
	}

	cout << endl << "Drone distance error Y direction: ";
	for (auto it = (*distanceErrorY).begin(); it != (*distanceErrorY).end(); ++it)
	{
		cout << ' ' << *it;
	}
	cout << endl << endl;











	//Assign values to Drone objects
	
	vector<Drone> *allOfTheDrones = new vector<Drone>;

	Drone* tempDrone = new Drone(noCells);
	for (int i = 0; i < noDrones + 1; i++)
	{
		(*allOfTheDrones).push_back(*tempDrone);
	}


	for (int i = 1; i < noDrones + 1; i++)
	{
		(*allOfTheDrones).at(i);
		(*allOfTheDrones).at(i).setDeadReckoningErrorX((*distanceErrorX).at(i-1));
		(*allOfTheDrones).at(i).setDeadReckoningErrorY((*distanceErrorY).at(i-1));
	}


	//Test inputs for Lij
	int Lij = 0;

	for (int i = 1; i < noDrones + 1; i++)
	{
		for (int j = 0; j < noCells; j++)
		{
			inFile >> Lij;
			(*allOfTheDrones).at(i).setLij(Lij, j);
			(*allOfTheDrones).at(i).setSpeed(speedMetersPerSecond, j);
			(*allOfTheDrones).at(i).setLengthDividedBySpeed(Lij, speedMetersPerSecond, j);
		}
		(*allOfTheDrones).at(i).setTheta((*thetaForEachDrone).at(i - 1));
	}
	
	//Display Lij
	for (int i = 0; i < noDrones + 1; i++)
	{
		cout << "Drone " << i + 1 << endl;
		for (int j = 0; j < noCells; j++)
		{
			(*allOfTheDrones).at(i).displayLij(j);
		}
		cout << endl;
	}
	


	
	int k = 0;


	double changeInX = 1;
	double changeInY = 1;

	int counter = 0;
	while (changeInY > .001)
	{
		k = k + 1;

		for (int i = 1; i < noDrones + 1; i++)
		{
			(*allOfTheDrones).at(i).calculateNextDroneXFlow((*allOfTheDrones).at(i - 1));
			(*allOfTheDrones).at(i).calculateNextDroneYFlow((*allOfTheDrones).at(i - 1));
		}

		for (int i = 1; i < noDrones + 1; i++)
		{
			(*allOfTheDrones).at(i).updateLij((*allOfTheDrones).at(noDrones));
		}
		changeInX = (*allOfTheDrones).at(noDrones-1).updateRateOfChangeX((*allOfTheDrones).at(0));
		changeInY = (*allOfTheDrones).at(noDrones-1).updateRateOfChangeY((*allOfTheDrones).at(0));

		cout << "ChangeInX: " << changeInX << endl;
		cout << "ChangeInY:" << changeInY << endl;

		(*allOfTheDrones).at(0).updateXFlow(noCells, (*allOfTheDrones).at(noDrones - 1));
		(*allOfTheDrones).at(0).updateYFlow(noCells, (*allOfTheDrones).at(noDrones - 1));
		counter++;
	}

	
	return 0;
}
