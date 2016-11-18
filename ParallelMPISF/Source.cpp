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

#include <iostream>
#include <fstream>
using namespace std;

int myRank, commSize;

void LoadData(vector<vector<Vector2>> &obstacles, vector<Vector2> &agentsPositions);
void SendObstacle(vector<Vector2> obstacle);
vector<Vector2> ReceiveObstacle();
void SendAgentPosition(Vector2 agentsPosition);
Vector2 ReceiveAgentPosition();
vector<pair<Vector2, Vector2>> DivideModelingArea(const vector<vector<Vector2>> &obstacles, float minimalWidth, float minimalHeight);
pair<Vector2, Vector2> CreateModelingArea(vector<vector<Vector2>> &obstacles, Vector2 minPoint, Vector2 maxPoint, float borderWidth);
int SendAgent(MPIAgent agent, int dest);
float GenerateRandomBetween(float LO, float HI);
void SaveObstaclesToJSON(vector<vector<Vector2>> obstacles, string path);

using namespace SF;

struct nodeAgent {
	bool isDeleted;
	size_t _nodeID;
	size_t _agentID;
	nodeAgent() : _nodeID(-1), _agentID(-1), isDeleted(false) { }
	nodeAgent(int nodeID, int agentID) : _nodeID(nodeID), _agentID(agentID), isDeleted(false) { }
};

int main(int argc, char* argv[])
{
	MPI_Init(&argc, &argv);
	MPI_Comm_rank(MPI_COMM_WORLD, &myRank);
	MPI_Comm_size(MPI_COMM_WORLD, &commSize);
	vector<pair<Vector2, Vector2>> modelingAreas;

	if (argc != 6)
	{
		if(myRank == 0)
		{
			cout << "Error! Invalid number of parameters: " << endl << " min_x min_y max_x max_y agent_calc_radius" << endl;
		}
		MPI_Finalize();
		return 0;
	}

	std::map<long long, nodeAgent> AgentsIDMap;
	long long totalAgentsIDs = 0; 
	std::map<long long, Vector2> AgentsPositions;
	SFSimulator simulator;

#pragma region AgentPropertyConfig MPI_BCasting
	unsigned char* serializedDefaultAgentConfig;
	AgentPropertyConfig* defaultAgentConfig;
	size_t buffSize = 0;
	if (myRank == 0)
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
	if (myRank != 0)
	{
		serializedDefaultAgentConfig = new unsigned char[buffSize];
	}

	MPI_Bcast(serializedDefaultAgentConfig, buffSize, MPI_UNSIGNED_CHAR, 0, MPI_COMM_WORLD);
	if (myRank != 0)
	{
		defaultAgentConfig = AgentPropertyConfig::Deseriaize(serializedDefaultAgentConfig);
		simulator.setAgentDefaults(*defaultAgentConfig);
	}

	delete serializedDefaultAgentConfig;

	//cout << "MyRank: " << myRank << " Properties: " << endl;
	//defaultAgentConfig->PrintDefaultProperties();

#pragma endregion AgentPropertyConfig MPI_BCasting

#pragma region Modeling area partitioning
	vector<vector<Vector2>> obstacles;
	vector<Vector2> agentsPositions;

	Vector2 p1(atoi(argv[1]), atoi(argv[2])); //Minimal point of modeling area
	Vector2 p2(atoi(argv[3]), atoi(argv[4])); //Maximal point of modeling area
	pair<Vector2, Vector2> globalArea = CreateModelingArea(obstacles, p1, p2, 1); //Create obstacle around modeling area (rectangle)

	LoadData(obstacles, agentsPositions);

	//Agent radius where it interact with anothers
	float minimalWidth = atoi(argv[5]);
	float minimalHeight = atoi(argv[5]);

	modelingAreas = DivideModelingArea(obstacles, minimalWidth, minimalHeight);

	if(myRank == 0)
	{
		SaveObstaclesToJSON(obstacles, "Obstacles.txt");
	}

	MPI_Barrier(MPI_COMM_WORLD);
	if (myRank != 0)
	{
		if (myRank <= modelingAreas.size())
		{
			cout << "I'm rank: " << myRank << " my area: x= " << modelingAreas[myRank - 1].first.x() << " y= " << modelingAreas[myRank - 1].first.y() << " x= " << modelingAreas[myRank - 1].second.x() << " y= " << modelingAreas[myRank - 1].second.y() << endl;
		}
		else
		{
			cout << "My rank:" << myRank << " Not enough areas for me" << endl;
		}
	}
#pragma endregion Modeling area partitioning

#pragma region Broadcasting generated agents

		if(myRank == 0)
		{
			for (int i = 0; i < agentsPositions.size(); i++)
			{
				int destinationNode = 0;
				bool IsPOintAdded = false;
				for(int a = 0; a < modelingAreas.size(); a++) //Choosing nodes
				{
					if(agentsPositions[i].x() >= modelingAreas[a].first.x() && agentsPositions[i].x() <= modelingAreas[a].second.x())
					{
						if(agentsPositions[i].y() >= modelingAreas[a].first.y() && agentsPositions[i].y() <= modelingAreas[a].second.y())
						{
							destinationNode = a + 1;
							MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
							//cout << "rank: " << myRank << " Sending to: " << destinationNode << endl;

							//Sending agent position
							float x = agentsPositions[i].x();
							float y = agentsPositions[i].y();
							MPI_Send(&x, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
							MPI_Send(&y, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
							size_t newAgentID = 0;
							//cout << "rank: " << myRank << " Position " << x << ":" << y << " to " << destinationNode << " sended."<< endl;
							//cout << "rank: " << myRank << " Receiving new ID from: " << destinationNode << endl;
							MPI_Recv(&newAgentID, 1, MPI_UNSIGNED, destinationNode, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE); //Receiving new agent ID
							//cout << "rank: " << myRank << " new agent id received: " << newAgentID << endl;
							AgentsIDMap[totalAgentsIDs] = nodeAgent(destinationNode, newAgentID);
							totalAgentsIDs++;
							cout << "rank: " << myRank << " Position " << x << ":" << y << " Global ID: "<< totalAgentsIDs - 1  << " Node: "<< AgentsIDMap[totalAgentsIDs - 1]._nodeID << " ID: " << AgentsIDMap[totalAgentsIDs - 1]._agentID << endl;
							IsPOintAdded = true;
						}
					}
				}
				if (!IsPOintAdded)
				{
					cout << "rank: " << myRank << " Position: " << agentsPositions[i].x() << ":" << agentsPositions[i].y() << " wasnt added" << endl;
				}
			}
			int negativeValueToStopListening = -1;
			MPI_Bcast(&negativeValueToStopListening, 1, MPI_INT, 0, MPI_COMM_WORLD);
		}
		else
		{
			int receivedIndex = 0;
			do
			{
				MPI_Bcast(&receivedIndex, 1, MPI_INT, 0, MPI_COMM_WORLD);
				if (receivedIndex == myRank)
				{
					//cout << "rank: " << myRank << " Receiving agent position" << endl;
					float x, y;
					MPI_Recv(&x, 1, MPI_FLOAT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					MPI_Recv(&y, 1, MPI_FLOAT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					//cout << "rank: " << myRank << " Position " << x << ":" << y << " received." << endl;
					size_t agentID = simulator.addAgent(Vector2(x, y));
					//cout << "rank: " << myRank << " new agend added to simulator. ID: " << agentID << endl;
					MPI_Send(&agentID, 1, MPI_INT, 0, 0, MPI_COMM_WORLD); //Sending back new agent ID
					//cout << "rank: " << myRank << " ID of new agent sent to main node." << endl;
				}
			} 
			while (receivedIndex > 0);
		}
	
	//agentsPositions
	//Vector2 agentPosition(5, 3);
	//simulator.setAgentDefaults(serializedDefaultAgentConfig);
	//auto aID = simulator.addAgent(agentPosition);
	//Agent* agent = simulator.getAgent(0);

	//MPIAgent mpiAgent1(agent);
	//unsigned char* array = mpiAgent1.SerializeAgent();

	//MPIAgent mpiAgent2;
	//mpiAgent2.DeserializeAgent(array);

	//simulator.addAgent(mpiAgent2.agent);
#pragma endregion Broadcasting generated agents

#pragma region Printing agents info on each node
	MPI_Barrier(MPI_COMM_WORLD);

	if(myRank == 0)
	{
		cout << "My rank: " << myRank << endl;
		cout << "Total agents count: " << AgentsIDMap.size() << endl;

		typedef map<long long, Vector2>::iterator it_type;
		for (it_type it = AgentsPositions.begin(); it != AgentsPositions.end(); it++) {
			cout << "Global ID: " << it->first << " Node: " << AgentsIDMap[it->first]._nodeID << " Local ID: " << AgentsIDMap[it->first]._agentID << " x: " << it->second.x() << " y: " << it->second.y() << endl;
		}
	}
	MPI_Barrier(MPI_COMM_WORLD);

	for(int i = 1; i < commSize; i++)
	{
		if(myRank == i)
		{
			cout << "Rank: " << myRank << endl;
			simulator.printAgentsInfo();
		}
		MPI_Barrier(MPI_COMM_WORLD);
	}

	cout << "MyRank: " << myRank << endl;
#pragma endregion Printing agents info on each node

	int iterationNum = 1000;
	int agentsCount = AgentsIDMap.size();

	std::fstream agentsPositionsFile;

	if(myRank == 0)
	{
		agentsPositionsFile.open("AgentsPositions.txt", ios::out | ios::trunc);
		agentsPositionsFile << "[" << endl;
	}


	for (int iter = 0; iter < iterationNum; iter++)
	{
		
		//if(myRank == 0)
		//{
		//	cout << "iteration: " << iter << endl;
		//}

		int destinationNode = 0;
		size_t agentId = -1;
		float xVel = 0;
		float yVel = 0;

		if (myRank == 0)
		{
			agentsPositionsFile << "{\"iteration\": \"" << iter << "\"," << endl;
			//Calculating velocity vectors and send it to nodes
			for (std::map<long long, nodeAgent>::iterator it = AgentsIDMap.begin(); it != AgentsIDMap.end(); ++it)
			{
				agentId = it->second._agentID;
				destinationNode = it->second._nodeID;
				if (it->first < agentsCount / 2) //Agents from zone A
				{
					if (AgentsPositions[it->first].x() <= 175)
					{
						xVel = 1;
						yVel = 0;
					}
					else //if they reached their destination
					{
						xVel = 0;
						yVel = 0;
					}
				}
				else //Agents from zone B
				{
					if (AgentsPositions[it->first].x() >= 25)
					{
						xVel = -1;
						yVel = 0;
					}
					else //if they reached their destination
					{
						xVel = 0;
						yVel = 0;
					}
				}

				//cout << "Sending new velocities to " << destinationNode << " aID: " << agentId << " xVel: " << xVel << " yVel: " << yVel << endl;
				MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
				MPI_Send(&agentId, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
				MPI_Send(&xVel, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
				MPI_Send(&yVel, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
			}

			//End of broadcasting
			destinationNode = -1;
			MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
			//cout << "Sending negative to stop velocities broadcasting" << endl;

			//Cheking crossing edges of areas

			MPI_Barrier(MPI_COMM_WORLD);
			//cout << "Rank: " << myRank << " Barrier reached " << endl;

			//Requesting agents new positions
			for (int areas = 0; areas < modelingAreas.size(); areas++)
			{
				int nodeID = areas + 1;
				vector<size_t> agentsIDs;
				for (std::map<long long, nodeAgent>::iterator it = AgentsIDMap.begin(); it != AgentsIDMap.end(); ++it)
				{
					if (it->second._nodeID == (nodeID)) //getting agents from current zone
					{
						agentsIDs.push_back(it->second._agentID);
					}
				}

				int requestAgentsCount = agentsIDs.size();
				MPI_Send(&requestAgentsCount, 1, MPI_INT, nodeID, 0, MPI_COMM_WORLD);
				//cout << "Rank: " << myRank << " requested agents count: " << requestAgentsCount << endl;
				//cout << "Rank: " << myRank << " requested agents IDs: " << requestAgentsCount << endl;
				//for (int i = 0; i < agentsIDs.size(); i++)
				//{
				//	cout << agentsIDs[i] << endl;
				//}
				for(int t = 0; t < requestAgentsCount; t++)
				{
					MPI_Send(&agentsIDs[t], 1, MPI_UNSIGNED, nodeID, 0, MPI_COMM_WORLD);
				}

				//cout << "Rank: " << myRank << " vector of IDs sent: " << requestAgentsCount << endl;

				for (int i = 0; i < requestAgentsCount; i++)
				{
					size_t agentID = 0;
					float xPos = 0;
					float yPos = 0;
					MPI_Recv(&agentID, 1, MPI_UNSIGNED, areas + 1, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					MPI_Recv(&xPos, 1, MPI_FLOAT, nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					MPI_Recv(&yPos, 1, MPI_FLOAT, nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

					for (std::map<long long, nodeAgent>::iterator it = AgentsIDMap.begin(); it != AgentsIDMap.end(); ++it)
					{
						if (it->second._nodeID == nodeID && it->second._agentID == agentID) //getting agents from current zone
						{
							AgentsPositions[it->first] = Vector2(xPos, yPos);

						}
					}
				}
			}

			MPI_Barrier(MPI_COMM_WORLD);
			agentsPositionsFile << "\"agents\": [" << endl;
			bool notFirseAgentInSection = false;
			for (std::map<long long, nodeAgent>::iterator it = AgentsIDMap.begin(); it != AgentsIDMap.end(); ++it)
			{
				if(notFirseAgentInSection)
				{
					agentsPositionsFile << "," << endl;
				}

				if (!it->second.isDeleted)
				{
					agentsPositionsFile << "{" << endl;
					agentsPositionsFile << "\t\"agentID\" : \"" << it->first << "\"," <<endl;
					agentsPositionsFile << "\t\"X\" : \"" << AgentsPositions[it->first].x() << "\"," << endl;
					agentsPositionsFile << "\t\"Y\" : \"" << AgentsPositions[it->first].y() << "\"" << endl;
					//agentsPositionsFile << it->first << "\t" << AgentsPositions[it->first].x() << "\t" << AgentsPositions[it->first].y() << "\t" << endl;
					agentsPositionsFile << "}";
				}
				notFirseAgentInSection = true;
			}
			agentsPositionsFile << "\n]" << endl;
		}
		else
		{
			//Receiving agent velocity vectors
			do
			{
				MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD); //Check if it information about agent on this node
				if (destinationNode == myRank)
				{
					MPI_Recv(&agentId, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					MPI_Recv(&xVel, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					MPI_Recv(&yVel, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					simulator.setAgentVelocity(agentId, Vector2(xVel, yVel));
				}
				//cout << "Rank: " << myRank << " destination node: " << destinationNode << endl;
			} while (destinationNode > 0);

			//cout << "Rank: " << myRank << " negative destination received." << endl;

			//Do simulation step
			simulator.doStep();
			//cout << "Rank: " << myRank << " step performed." << endl;

			//Checking placing agents of the edge of modeling area
			//and moving to another node
			//simulator.GetAgentsVector()

			//int agentsToMove = 0;
			//MPI_Send(&agentsToMove, 1, MPI_INT, 0, 0, MPI_COMM_WORLD);


			MPI_Barrier(MPI_COMM_WORLD);
			//Sending new positions to main node

			//Requesting vector with requesting agents IDs
			int requestedAgentsCount;
			vector<size_t> requestedAgentsIDs;
			MPI_Recv(&requestedAgentsCount, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
			//cout << "Rank: " << myRank << " count of requested agents received: " << requestedAgentsCount << endl;

			for(int t  = 0; t < requestedAgentsCount; t++)
			{
				int requestedID = 0;
				MPI_Recv(&requestedID, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
				requestedAgentsIDs.push_back(requestedID);
			}

			//cout << "Rank: " << myRank << " vector with requested agents IDs received." << endl;
			//cout << "Rank: " << myRank << " requested agents: " << requestedAgentsIDs.size()  << endl;
			//for(int i = 0; i < requestedAgentsIDs.size(); i++)
			//{
			//	cout << requestedAgentsIDs[i] << endl;
			//}
			//cout << "Rank: " << myRank << " after loop " << endl;

			std::map<size_t, Vector2> agentsNewPositions;
			for (int i = 0; i < requestedAgentsIDs.size(); i++)
			{
				//cout << "Rank: " << myRank << " requesting position of agent " << endl;
				agentsNewPositions[requestedAgentsIDs[i]] = simulator.getAgentPosition(requestedAgentsIDs[i]);
			}
			//cout << "Rank: " << myRank << " Agents new positions from simulator obtained." << endl;

			//Sending coordinates of reguested agents
			for (std::map<size_t, Vector2>::iterator it = agentsNewPositions.begin(); it != agentsNewPositions.end(); ++it)
			{
				float xPos = it->second.x();
				float yPos = it->second.y();
				MPI_Send(&it->first, 1, MPI_INT, 0, 0, MPI_COMM_WORLD);
				MPI_Send(&xPos, 1, MPI_FLOAT, 0, 0, MPI_COMM_WORLD);
				MPI_Send(&yPos, 1, MPI_FLOAT, 0, 0, MPI_COMM_WORLD);
			}
			//cout << "Rank: " << myRank << " Agents new positions sended." << endl;

			MPI_Barrier(MPI_COMM_WORLD);
		}
		
		if(myRank == 0)
		{
			agentsPositionsFile << "}";
			if(iter < iterationNum - 1)
			{
				agentsPositionsFile << ",";
			}
		}

	}

	if (myRank == 0)
	{
		agentsPositionsFile << "]" << endl;
		agentsPositionsFile.close();
	}


	delete defaultAgentConfig;

	MPI_Finalize();

	return 0;
}


void LoadData(vector<vector<Vector2>> &obstacles, vector<Vector2> &agentsPositions)
{
	//int rank;
	MPI_Comm_rank(MPI_COMM_WORLD, &myRank);
	//Agents generating zones
	Vector2 zomeAMinPoint(0, 0);
	Vector2 zomeAMaxPoint(50, 100);
	Vector2 zomeBMinPoint(150, 0);
	Vector2 zomeBMaxPoint(200, 100);

	//Loading agents from file



	//Generating agents at main node
	if (myRank == 0)
	{
		srand(time(nullptr));

		//random agents in zone A
		for (int i = 0; i < 10; i++)
		{
			Vector2 agentPosition(GenerateRandomBetween(zomeAMinPoint.x(), zomeAMaxPoint.x()), GenerateRandomBetween(zomeAMinPoint.y(), zomeAMaxPoint.y()));
				//zomeAMinPoint.x() + rand() % (int)zomeAMaxPoint.x(), zomeAMinPoint.y() + rand() % (int)zomeAMaxPoint.y());
			agentsPositions.push_back(agentPosition);
		}

		//random agents in zone B
		for (int i = 0; i < 10; i++)
		{
			Vector2 agentPosition(GenerateRandomBetween(zomeBMinPoint.x(), zomeBMaxPoint.x()), GenerateRandomBetween(zomeBMinPoint.y(), zomeBMaxPoint.y()));
				//zomeBMinPoint.x() + rand() % (int)zomeBMaxPoint.x(), zomeBMinPoint.y() + rand() % (int)zomeBMaxPoint.y());
			agentsPositions.push_back(agentPosition);
		}
	}

	//Loading obstacles from file

	//Broadcasting obstacles that loaded from file
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

float GenerateRandomBetween(float LO, float HI)
{
	return LO + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HI - LO)));
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

//int SendAgent(MPIAgent agent, int dest)
//{
//	char *buffer;
//	int position = 0;nodeAgent
//
//	//MPI_Pack( acceleration_, position, MPI_FLOAT, &buffer);
//
//	return 0;
//}

void SaveObstaclesToJSON(vector<vector<Vector2>> obstacles, string path)
{
	std::fstream agentsPositionsFile;
	agentsPositionsFile.open(path, ios::out | ios::trunc);
	agentsPositionsFile << "[" << endl;

	for(int obst = 0; obst < obstacles.size(); obst++)
	{
		agentsPositionsFile << "{" << endl;
		agentsPositionsFile << "\"obstacleID\": " << obst <<","<< endl;
		agentsPositionsFile << "\"points\": [" << endl;
		for(int point = 0; point < obstacles[obst].size(); point++)
		{
			agentsPositionsFile << "{" << endl;
			agentsPositionsFile << "\"X\" : " << obstacles[obst][point].x()<< "," << endl;
			agentsPositionsFile << "\"Y\" : " << obstacles[obst][point].y() << endl;
			if(point < obstacles[obst].size() - 1)
			{
				agentsPositionsFile << "}," << endl;
			}
			else
			{
				agentsPositionsFile << "}" << endl;
			}
		}
		agentsPositionsFile << "]" << endl;
		if(obst < obstacles.size() - 1)
		{
			agentsPositionsFile << "}," << endl;
		}
		else
		{
			agentsPositionsFile << "}" << endl;
		}
	}

	agentsPositionsFile << "]" << endl;
	agentsPositionsFile.close();
}