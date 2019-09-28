#include <iostream>
#include <vector>
#include <math.h>
using namespace std;

class Position
{
	protected:
		int xPosition;	
		int yPosition;
		int time;
	public:
		Position();
		Position(int x, int y, int t);
		void setXPosition(int x);
		void setYPosition(int y);
		void setTime(int t);
		int getXPosition();
		int getYPosition();
		int getTime();
};

Position::Position() 
{
	xPosition = NULL;
	yPosition = NULL;
	time = NULL;
}

Position::Position(int x, int y, int t)
{
	xPosition = x;
	yPosition = y;
	time = t;
}

void Position::setXPosition(int x)
{
	xPosition = x;
}

void Position::setYPosition(int y) 
{
	yPosition = y;
}

void Position::setTime(int t)
{
	time = t;
}

int Position::getXPosition() 
{
	return xPosition;
}

int Position::getYPosition()
{
	return yPosition;
}

int Position::getTime()
{
	return time;
}




class FlowEstimate
{
protected:
	int xFlow;
	int yFlow;
public:
	FlowEstimate();
	FlowEstimate(int x, int y);
	int getXFlow();
	int getYFlow();
};

FlowEstimate::FlowEstimate() 
{
	xFlow = NULL;
	yFlow = NULL;
}

FlowEstimate::FlowEstimate(int x, int y)
{
	xFlow = x;
	yFlow = y;
}



int FlowEstimate::getXFlow()
{
	return xFlow;
}

int FlowEstimate::getYFlow()
{
	return yFlow;
}





class ReckoningEstimate {
	protected:
		FlowEstimate fe;
		int speed;
		int switchingSignal;
		int compassHeading;
		int reckonedXPosition;
		int reckonedYPosition;
	public:
		ReckoningEstimate();
		ReckoningEstimate(FlowEstimate f, int s, int ch, int ss);
		void calculatePredictedPosition(int s, int e);
		int getReckonedXPosition();
		int getReckonedYPosition();
};

ReckoningEstimate::ReckoningEstimate() {
	speed = NULL;
	compassHeading = NULL;
	switchingSignal = NULL;
}


ReckoningEstimate::ReckoningEstimate(FlowEstimate f, int s, int ch, int ss)
{
	fe = f;
	speed = s;
	compassHeading = ch;
	switchingSignal = ss;
}

void ReckoningEstimate::calculatePredictedPosition(int startingTime, int endingTime) 
{
	int totalTime;
	totalTime = endingTime - startingTime;
	reckonedXPosition = totalTime * (speed *(cos(compassHeading)) + fe.getXFlow());
	reckonedYPosition = totalTime * (speed *(sin(compassHeading)) + fe.getYFlow());
}



int ReckoningEstimate::getReckonedXPosition()
{
	return reckonedXPosition;
}

int ReckoningEstimate::getReckonedYPosition()
{
	return reckonedYPosition;
}







int main() 
{

	Position* actual;
	Position* reckoned;
	int x, y, t, theta, s;
	int switchingSignal = 1;
	int deltaT = .1;
	
	vector<FlowEstimate>* testFlows = new vector<FlowEstimate>;
	FlowEstimate* initialGuess = new FlowEstimate(1, 1);
	(*testFlows).push_back((*initialGuess));

	ReckoningEstimate* re;


	cin >> x >> y >> t >> theta >> s;

	re = new ReckoningEstimate((*testFlows).back(), s, theta, switchingSignal);
	(*re).calculatePredictedPosition(t,t+1);
	reckoned = new Position((*re).getReckonedXPosition(), (*re).getReckonedYPosition(),t+5);


	int tempXFlow, tempYFlow;


	while (!cin.eof()) 
	{
		cin >> x >> y >> t >> theta >> s;

		actual = new Position(x, y, t);

		
		//Calculate new flow
		//store new flow in vector




		re = new ReckoningEstimate((*testFlows).back(), s, theta, switchingSignal);
		reckoned = new Position((*re).getReckonedXPosition(), (*re).getReckonedYPosition(), t);



		

		

		

	}
	

	return 0;
}