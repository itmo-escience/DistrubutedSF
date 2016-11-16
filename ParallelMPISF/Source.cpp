/*
* Copyright (c) 2004-2006 The Trustees of Indiana University and Indiana
*                         University Research and Technology
*                         Corporation.  All rights reserved.
* Copyright (c) 2006      Cisco Systems, Inc.  All rights reserved.
*
* Sample MPI "hello world" application in C
*/

#include <stdio.h>
#include "mpi.h"
#include "Slave.h"
#include "Master.h"

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <iostream>
#include "../social-phys-lib-private/SF/MPIAgent.h"

int myRank, commSize;

void LoadData(vector<vector<Vector2>> &obstacles, vector<Vector2> &agentsPositions);
void SendObstacle(vector<Vector2> obstacle);
vector<Vector2> ReceiveObstacle();
void SendAgentPosition(Vector2 agentsPosition);
Vector2 ReceiveAgentPosition();
vector<pair<Vector2, Vector2>> DivideModelingArea(const vector<vector<Vector2>> &obstacles, float minimalWidth, float minimalHeight);
pair<Vector2, Vector2> localArea;
pair<Vector2, Vector2> CreateModelingArea(vector<vector<Vector2>> &obstacles, Vector2 minPoint, Vector2 maxPoint, float borderWidth);
int SendAgent(MPIAgent agent, int dest);

using namespace SF;

struct nodeAgent {
	bool isDeleted;
	size_t _nodeID;
	size_t _agentID;
	nodeAgent(int nodeID, int agentID) : _nodeID(nodeID), _agentID(agentID), isDeleted(false) { }
};

int main(int argc, char* argv[])
{	
	MPI_Init(&argc, &argv);
	MPI_Comm_rank(MPI_COMM_WORLD, &myRank);
	MPI_Comm_size(MPI_COMM_WORLD, &commSize);
	pair<Vector2, Vector2> myArea;

	if(argc != 6)
	{
		cout << "Error! Invalid number of parameters: " << endl << " min_x min_y max_x max_y agent_calc_radius" << endl;
		cout << "argc: " << argc << endl;
		return 0;
	}

	std::map<size_t, nodeAgent> AgentsIdMap;
	SFSimulator simulator;

#pragma region AgentPropertyConfig MPI_BCasting
	unsigned char* serializedDefaultAgentConfig;
	AgentPropertyConfig* defaultAgentConfig;
	size_t buffSize = 0;
	if(myRank == 0)
	{
		defaultAgentConfig = new AgentPropertyConfig
		(
			15.f, 	//	NeighborDist = 15f,
			10, 	//	MaxNeighbors = 10,
			5.f, 	//	TimeHorizon = 5f,
			0.2f,	//	Radius = 0.2f,
			2.0f,	//	MaxSpeed = 2.0f,
			1.f, 	//	float force,
			0.5f,	//	float accelerationCoefficient,
			1.f, 	//	float relaxationTime,
			0.19f,	//	float repulsiveAgent,
			46, 	//	float repulsiveAgentFactor,
			4.f, 	//	float repulsiveObstacle,
			0.32f, 	//	float repulsiveObstacleFactor,
			0.0f, 	//	float obstacleRadius,
			0.f,	//	float platformFactor,
			0.25f,	//	float perception,
			1,		//	float friction,
			Vector2(0, 0)	//	Vector2 velocity
		);
		
		serializedDefaultAgentConfig = defaultAgentConfig->Serialize();
		memcpy(&buffSize, serializedDefaultAgentConfig, sizeof(buffSize));
	}
	
	MPI_Bcast(&buffSize, 1, MPI_INT, 0, MPI_COMM_WORLD);
	if(myRank != 0)
	{
		serializedDefaultAgentConfig = new unsigned char[buffSize];
	}
	
	MPI_Bcast(serializedDefaultAgentConfig, buffSize, MPI_UNSIGNED_CHAR, 0, MPI_COMM_WORLD);
	if(myRank != 0)
	{
		defaultAgentConfig = AgentPropertyConfig::Deseriaize(serializedDefaultAgentConfig);
		simulator.setAgentDefaults(*defaultAgentConfig);
	}

	delete serializedDefaultAgentConfig;

	cout << "MyRank: " << myRank << " Properties: " << endl;
	defaultAgentConfig->PrintDefaultProperties();

#pragma endregion AgentPropertyConfig MPI_BCasting

#pragma region Modeling area partitioning
	vector<vector<Vector2>> obstacles;
	vector<Vector2> agentsPositions;

	Vector2 p1(atoi(argv[1]), atoi(argv[2]));
	Vector2 p2(atoi(argv[3]), atoi(argv[4]));
	pair<Vector2, Vector2> globalArea = CreateModelingArea(obstacles, p1, p2, 1);


	LoadData(obstacles, agentsPositions);

	float minimalWidth = atoi(argv[5]);
	float minimalHeight = atoi(argv[5]);
	vector<pair<Vector2, Vector2>> areas = DivideModelingArea(obstacles, minimalWidth, minimalHeight);

	float minX;
	float minY;
	float maxX;
	float maxY;

	if (myRank == 0)
	{
		cout << "min_x: " << globalArea.first.x() << " min_y: " << globalArea.first.y() << " max_x: " << globalArea.second.x() << " max_y: " << globalArea.second.y() << endl;
		cout << "areas: " << areas.size() << endl;
		cout << "minimalWidth: " << minimalWidth << " minimalHeight: " << minimalHeight << endl;

		for (int i = 1; i <= areas.size(); i++)
		{
			//cout << "I'm : " << myRank << " sending to: " << i << endl;
			minX = areas[i - 1].first.x();
			MPI_Send(&minX, 1, MPI_FLOAT, i, 1, MPI_COMM_WORLD);

			minY = areas[i - 1].first.y();
			MPI_Send(&minY, 1, MPI_FLOAT, i, 1, MPI_COMM_WORLD);

			maxX = areas[i - 1].second.x();
			MPI_Send(&maxX, 1, MPI_FLOAT, i, 1, MPI_COMM_WORLD);

			maxY = areas[i - 1].second.y();
			MPI_Send(&maxY, 1, MPI_FLOAT, i, 1, MPI_COMM_WORLD);
		}
	}
	else
	{
		if (myRank <= areas.size())
		{
			cout << "I'm : " << myRank << " receiving from: " << 0 << " areas: " << areas.size() << endl;
			MPI_Recv(&minX, 1, MPI_FLOAT, 0, 1, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
			MPI_Recv(&minY, 1, MPI_FLOAT, 0, 1, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
			MPI_Recv(&maxX, 1, MPI_FLOAT, 0, 1, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
			MPI_Recv(&maxY, 1, MPI_FLOAT, 0, 1, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

			Vector2 minPoint(minX, minY);
			Vector2 maxPoint(maxX, maxY);
			myArea = make_pair(minPoint, maxPoint);
		}
		//else
		//{
		//	cout << "I'm : " << myRank << " not enough areas for me." << endl;
		//}
	}

	MPI_Barrier(MPI_COMM_WORLD);
	if (myRank != 0 && myRank <= areas.size())
	{
		cout << "I'm rank: " << myRank << " my area: x= " << myArea.first.x() << " y= " << myArea.first.y() << " x= " << myArea.second.x() << " y= " << myArea.second.y() << endl;
	}

#pragma endregion Modeling area partitioning


	Vector2 agentPosition(5, 3);
	simulator.setAgentDefaults(serializedDefaultAgentConfig);
	auto aID = simulator.addAgent(agentPosition);
	Agent* agent = simulator.getAgent(0);

	MPIAgent mpiAgent1(agent);
	unsigned char* array = mpiAgent1.SerializeAgent();
	
	MPIAgent mpiAgent2;
	mpiAgent2.DeserializeAgent(array);

	simulator.addAgent(mpiAgent2.agent);

	//printf("Hello, world, I am %d of %d, (%s, %d)\n", myRank, commSize, version, len);

	//if(commSize < 2)
	//{
	//	cout << "Error! At least 2 nodes are required" << endl;
	//	return 0;
	//}




	//if(myRank > 0)
	//{
	//	int myAreaIndex = myRank - 1;
	//	if (areas.size() > myAreaIndex)
	//	{
	//		cout << "My area is: " << endl;
	//		cout << "x1: " << areas[myAreaIndex].first.x() << " y1: " << areas[myAreaIndex].first.y() << " x2: " << areas[myAreaIndex].second.x() << " y2: " << areas[myAreaIndex].second.y() << endl;
	//	}
	//}

	//if(myRank == 0)
	//{
	//	for (int i = 0; i < areas.size(); i++)
	//	{
	//		cout << "Area " << i << ":" << "x1: " << areas[i].first.x() << " y1: " << areas[i].first.y() << " x2: " << areas[i].second.x() << " y2: " << areas[i].second.y() << endl;
	//	}
	//}

	//SFSimulator simulator;
	//Vector2 agentPosition(5, 3);
	//simulator.addAgent(agentPosition);
	//Agent* agent = simulator.getAgent(0);
	//
	//MPIAgent mpiAgent(agent);
	//unsigned char* array = mpiAgent.SerializeAgent();

	//cout << "Im rank: " << rank <<  " obstacles count: " << obstacles.size() << endl;
	//for(int i = 0; i < obstacles.size(); i++)
	//{
	//	for(int j = 0; j < obstacles[i].size(); j++)
	//	{
	//		cout << "P[" << j << "] = " << obstacles[i][j].x() << ":" << obstacles[i][j].y() << endl;
	//	}
	//}

	//cout << "Im rank: " << rank << " agents count: " << agentsPositions.size() << endl;
	//for (int i = 0; i < agentsPositions.size(); i++)
	//{
	//		cout << "A[" << i << "] = " << agentsPositions[i].x() << ":" << agentsPositions[i].y() << endl;
	//}

	delete defaultAgentConfig;

	MPI_Finalize();

	return 0;
}


void LoadData(vector<vector<Vector2>> &obstacles, vector<Vector2> &agentsPositions)
{
	//int rank;
	MPI_Comm_rank(MPI_COMM_WORLD, &myRank);
	Vector2 zomeAMinPoint(0, 0);
	Vector2 zomeAMaxPoint(50, 100);
	Vector2 zomeBMinPoint(150, 0);
	Vector2 zomeBMaxPoint(200, 100);

	//Load from file
	//obstacles.clear();
	//agentsPositions.clear();

	srand(time(NULL));

	if (myRank == 0)
	{
		//random agents in zone A
		for (int i = 0; i < 10; i++)
		{
			Vector2 agentPosition(zomeAMinPoint.x() + rand() % (int)zomeAMaxPoint.x(), zomeAMinPoint.y() + rand() % (int)zomeAMaxPoint.y());
			agentsPositions.push_back(agentPosition);
		}

		//random agents in zone B
		for (int i = 0; i < 10; i++)
		{
			Vector2 agentPosition(zomeBMinPoint.x() + rand() % (int)zomeBMaxPoint.x(), zomeBMinPoint.y() + rand() % (int)zomeBMaxPoint.y());
			agentsPositions.push_back(agentPosition);
		}
	}

	int obstaclesCount = 0;
	int agentsCount = 0;
	if (myRank == 0)
	{
		obstaclesCount = obstacles.size();
		MPI_Bcast(&obstaclesCount, 1, MPI_INT, 0, MPI_COMM_WORLD);
		for(int i = 0; i < obstaclesCount; i++)
		{
			SendObstacle(obstacles[i]);
		}

		agentsCount = agentsPositions.size();
		MPI_Bcast(&agentsCount, 1, MPI_INT, 0, MPI_COMM_WORLD);
		for (int i = 0; i < agentsCount; i++)
		{
			SendAgentPosition(agentsPositions[i]);
		}
	}
	else
	{
		MPI_Bcast(&obstaclesCount, 1, MPI_INT, 0, MPI_COMM_WORLD);
		for (int i = 0; i < obstaclesCount; i++)
		{
			obstacles.push_back(ReceiveObstacle());
		}

		MPI_Bcast(&agentsCount, 1, MPI_INT, 0, MPI_COMM_WORLD);
		for (int i = 0; i < agentsCount; i++)
		{
			agentsPositions.push_back(ReceiveAgentPosition());
		}
	}
}

void SendObstacle(vector<Vector2> obstacle)
{
	int obstaclePointNum = obstacle.size();
	MPI_Bcast(&obstaclePointNum, 1, MPI_INT, 0, MPI_COMM_WORLD);

	for(int i = 0; i < obstaclePointNum; i++)
	{
		float pointX = obstacle[i].x();
		float pointY = obstacle[i].y();
		MPI_Bcast(&pointX, 1, MPI_FLOAT, 0, MPI_COMM_WORLD);
		MPI_Bcast(&pointY, 1, MPI_FLOAT, 0, MPI_COMM_WORLD);
	}
}

vector<Vector2> ReceiveObstacle()
{
	int obstaclePointNum = 0;
	MPI_Bcast(&obstaclePointNum, 1, MPI_INT, 0, MPI_COMM_WORLD);
	vector<Vector2> obstacle;

	for (int i = 0; i < obstaclePointNum; i++)
	{
		float pointX = 0;
		float pointY = 0;
		MPI_Bcast(&pointX, 1, MPI_FLOAT, 0, MPI_COMM_WORLD);
		MPI_Bcast(&pointY, 1, MPI_FLOAT, 0, MPI_COMM_WORLD);
		Vector2 point(pointX, pointY);
		obstacle.push_back(point);
	}

	return obstacle;
}

void SendAgentPosition(Vector2 agentsPosition)
{
	float pointX = agentsPosition.x();
	float pointY = agentsPosition.y();
	MPI_Bcast(&pointX, 1, MPI_FLOAT, 0, MPI_COMM_WORLD);
	MPI_Bcast(&pointY, 1, MPI_FLOAT, 0, MPI_COMM_WORLD);
}

Vector2 ReceiveAgentPosition()
{
	float pointX = 0;
	float pointY = 0;
	MPI_Bcast(&pointX, 1, MPI_FLOAT, 0, MPI_COMM_WORLD);
	MPI_Bcast(&pointY, 1, MPI_FLOAT, 0, MPI_COMM_WORLD);
	Vector2 agentPosition(pointX, pointY);

	return agentPosition;
}

//Dividing modeling area to subareas for each nodes. If subarea is smaller than minimal width or height it is not dividing more 
vector<pair<Vector2, Vector2>> DivideModelingArea(const vector<vector<Vector2>> &obstacles, float minimalWidth, float minimalHeight)
{
	float minX = INT_MAX;
	float minY = INT_MAX;
	float maxX = INT_MIN;
	float maxY = INT_MIN;
	
	for(int i = 0; i < obstacles.size(); i++)
	{
		for(int j = 0; j < obstacles[i].size(); j++)
		{
			if(obstacles[i][j].x() < minX)
			{
				minX = obstacles[i][j].x();
			}
			if (obstacles[i][j].y() < minY)
			{
				minY = obstacles[i][j].y();
			}
			if (obstacles[i][j].x() > maxX)
			{
				maxX = obstacles[i][j].x();
			}
			if (obstacles[i][j].y() > maxY)
			{
				maxY = obstacles[i][j].y();
			}
		}
	}

	vector<pair<Vector2, Vector2>> resultAreas;
	Vector2 pointA(minX, minY);
	Vector2 pointB(maxX, maxY);
	pair<Vector2, Vector2> pair_(pointA, pointB);
	resultAreas.push_back(pair_);

	//commSize = 5;

	if (commSize > 2)
	{
		bool tooSmallDivision = false;
		while (resultAreas.size() < (commSize - 1) && !tooSmallDivision)
		{
			int currIterationVectSize = resultAreas.size();
			bool areaWasDivided = false;
			for (int i = 0; i < currIterationVectSize; i++)
			{
				if (resultAreas[i].second.x() - resultAreas[i].first.x() <= resultAreas[i].second.y() - resultAreas[i].first.y()) //horizontal dividing
				{
					if((resultAreas[i].second.y() - resultAreas[i].first.y()) / 2 < minimalHeight)
					{
						continue;
					}
					float newMaxY = resultAreas[i].first.y() + ((resultAreas[i].second.y() - resultAreas[i].first.y()) / 2);
					Vector2 pointA2(resultAreas[i].first.x(), newMaxY);
					Vector2 pointB2(resultAreas[i].second.x(), resultAreas[i].second.y());
					pair<Vector2, Vector2> pair2(pointA2, pointB2);
					resultAreas.push_back(pair2);

					Vector2 pointA1(resultAreas[i].first.x(), resultAreas[i].first.y());
					Vector2 pointB1(resultAreas[i].second.x(), newMaxY);
					pair<Vector2, Vector2> pair1(pointA1, pointB1);
					resultAreas[i] = pair1;

					areaWasDivided = true;
				}
				else //Vertical dividing
				{
					if ((resultAreas[i].second.x() - resultAreas[i].first.x()) / 2 < minimalWidth)
					{
						continue;
					}
					float newMaxX = resultAreas[i].first.x() + ((resultAreas[i].second.x() - resultAreas[i].first.x()) / 2);
					Vector2 pointA2(newMaxX, resultAreas[i].first.y());
					Vector2 pointB2(resultAreas[i].second.x(), resultAreas[i].second.y());
					pair<Vector2, Vector2> pair2(pointA2, pointB2);
					resultAreas.push_back(pair2);

					Vector2 pointA1(resultAreas[i].first.x(), resultAreas[i].first.y());
					Vector2 pointB1(newMaxX, resultAreas[i].second.y());
					pair<Vector2, Vector2> pair1(pointA1, pointB1);
					resultAreas[i] = pair1;

					areaWasDivided = true;
				}

				if (resultAreas.size() >= commSize - 1)
				{
					break;
				}
			}

			if (!areaWasDivided)
			{
				tooSmallDivision = true;
			}
		}
	}

	return resultAreas;
}

pair<Vector2, Vector2> CreateModelingArea(vector<vector<Vector2>> &obstacles, Vector2 minPoint, Vector2 maxPoint, float borderWidth)
{
	vector<Vector2> obstacle;
	obstacle.push_back(minPoint);
	obstacle.push_back(Vector2(minPoint.x(), maxPoint.y()));
	obstacle.push_back(maxPoint);
	obstacle.push_back(Vector2(maxPoint.x(), minPoint.y()));
	obstacle.push_back(Vector2(minPoint.x() - borderWidth, minPoint.y()));
	obstacle.push_back(Vector2(minPoint.x() - borderWidth, minPoint.y() - borderWidth));
	obstacle.push_back(Vector2(maxPoint.x() + borderWidth, minPoint.y() - borderWidth));
	obstacle.push_back(Vector2(maxPoint.x() + borderWidth, maxPoint.y() + borderWidth));
	obstacle.push_back(Vector2(minPoint.x() - borderWidth, maxPoint.y() + borderWidth));
	obstacle.push_back(Vector2(minPoint.x() - borderWidth, minPoint.y()));
	obstacle.push_back(minPoint);

	obstacles.push_back(obstacle);

	float minX = INT_MAX;
	float minY = INT_MAX;
	float maxX = INT_MIN;
	float maxY = INT_MIN;

	for (int i = 0; i < obstacles.size(); i++)
	{
		for (int j = 0; j < obstacles[i].size(); j++)
		{
			if (obstacles[i][j].x() < minX)
			{
				minX = obstacles[i][j].x();
			}
			if (obstacles[i][j].y() < minY)
			{
				minY = obstacles[i][j].y();
			}
			if (obstacles[i][j].x() > maxX)
			{
				maxX = obstacles[i][j].x();
			}
			if (obstacles[i][j].y() > maxY)
			{
				maxY = obstacles[i][j].y();
			}
		}
	}

	Vector2 newMinPoint(minX, minY);
	Vector2 newMaxPoint(maxX, maxY);

	pair<Vector2, Vector2> globalArea(newMinPoint, newMaxPoint);
	return globalArea;

	// Border ():
	//      _______________________
	//     | ____________________  |
	//     | |                   | |
	//     | |                   | |
	//     | |                   | |
	//     |_|                   | |
	//      _                    | |
	//     | |___________________| |
	//     |_______________________|
	//
	//
}

int SendAgent(MPIAgent agent, int dest)
{
	char *buffer;
	int position = 0;

	//MPI_Pack( acceleration_, position, MPI_FLOAT, &buffer);

	return 0;
}