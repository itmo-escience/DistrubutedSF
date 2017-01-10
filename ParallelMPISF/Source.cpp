#include <mpi.h>
#include <stdio.h>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <iostream>
#include <fstream>
#include "../../ParallelMPISF/social-phys-lib-private/SF/include/MPIAgent.h"
#include <set>
#include "AgentOnNodeInfo.h"

using namespace std;
using namespace SF;

int myRank, commSize;
int totalAgentsCount;
SFSimulator* simulator;
pair<Vector2, Vector2> GlobalArea;
map<int, pair<Vector2, Vector2>> modelingAreas;
AgentPropertyConfig* defaultAgentConfig;
vector<vector<Vector2>> obstacles;

map<long long, AgentOnNodeInfo> AgentsIDMap; //Global ID, agent node info
map<int, map<long long, long long>> NodesAgentsMap;// node id, agent ID on node, gloabal ID;
long long totalAgentsIDs = 0; 
map<long long, Vector2> AgentsPositions;
int adjacentAreaWidth;
vector<map<long long, pair<Vector2, AgentOnNodeInfo>>> simulationData;

void LoadData(vector<vector<SF::Vector2> > &obstacles, vector<Vector2> &agentsPositions, pair<Vector2, Vector2> zoneA, pair<Vector2, Vector2> zoneB );
void SendObstacle(vector<Vector2> obstacle);
vector<Vector2> ReceiveObstacle();
void SendAgentPosition(Vector2 agentsPosition);
Vector2 ReceiveAgentPosition();
map<int, pair<Vector2, Vector2>> DivideModelingArea(const pair<Vector2, Vector2> &globalArea, int adjacentAreaWidth);
pair<Vector2, Vector2> CreateModelingArea(vector<vector<Vector2> > &obstacles, Vector2 minPoint, Vector2 maxPoint, float borderWidth);
int SendAgent(MPIAgent agent, int dest);
float GenerateRandomBetween(float LO, float HI);
void SaveObstaclesToJSON(vector<vector<Vector2> > obstacles, const string path);
void SavePartitionedAreasToJSON(map<int, pair <Vector2, Vector2> > modelingAreas, const string path, int adjacentAreaWidth);
void SaveSimDataToFile(string filename, vector<map<long long, pair<Vector2, AgentOnNodeInfo>>> simulationData);

void AgentPropertyConfigBcasting();
vector<Vector2> ModelingAreaPartitioning(char* argv[]);
void BcastingObstacles();
void BroadcastingGeneratedAgents(vector<Vector2> agentsPositions);
vector<Vector2> GenerateRandomAgentsPositions();

void SendNewVelocities();
void ExchangingByPhantoms();
void UpdateAgentsPositionOnMainNode();
void DoSimulationStep();
void AgentsShifting();
void SavingModelingData();

int main(int argc, char* argv[])
{
	MPI_Init(&argc, &argv);
	MPI_Comm_rank(MPI_COMM_WORLD, &myRank);
	MPI_Comm_size(MPI_COMM_WORLD, &commSize);

#pragma region ARGUMENTS TREATING
	if (argc != 7)
	{
		if(myRank == 0)
		{
			std::cout << "Error! Invalid number of parameters: " << endl << " min_x min_y max_x max_y agent_calc_radius totalAgentsCount" << endl;
		}

		MPI_Finalize();
		return 0;
	}

	if(myRank == 0)
	{
		std::cout << "CommSize: " << commSize << endl;
	}

	if(commSize < 2)
	{
		MPI_Finalize();
		return 0;
	}
#pragma endregion ARGUMENTS TREATING

	int startTime = clock(); //programm working start moment
	simulator = new SFSimulator();

	AgentPropertyConfigBcasting();
	vector<Vector2> agentsPositions = ModelingAreaPartitioning(argv);
	BcastingObstacles();
	BroadcastingGeneratedAgents(agentsPositions);

	int iterationNum = 150;
	int iterationTimeStart;

	for (int iter = 0; iter < iterationNum; iter++)
	{
		iterationTimeStart = clock();

		if(myRank == 0)
		{
			cout << "Iteration: " << iter << endl;	
		}

		SendNewVelocities();
		ExchangingByPhantoms(); //If some agents in adjacent areas
		DoSimulationStep();     //Workers perform simulation step
		UpdateAgentsPositionOnMainNode(); //Workers send agents new positions to main node
		AgentsShifting();		//If some agent crossed modeling subarea
		SavingModelingData();	//Main node put agents positions to list

		printf ("Iteration time: (%f seconds).\n",((float)clock() - iterationTimeStart)/CLOCKS_PER_SEC);
	}

	if (myRank == 0)
	{
		SaveSimDataToFile("simData.txt", simulationData);
	}

	int deletingStartTime = clock();
	delete defaultAgentConfig;
	delete simulator;
	printf ("Deleting time:  (%f seconds).\n",((float)clock() - deletingStartTime)/CLOCKS_PER_SEC);

	MPI_Finalize();
	cout << myRank << "After finalization at:" << clock()<< endl;

	int workTime = clock() - startTime;
	printf ("program working time: %d clicks (%f seconds).\n",workTime,((float)workTime)/CLOCKS_PER_SEC);

	return 0;
}

float GenerateRandomBetween(float LO, float HI)
{
	return LO + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HI - LO)));
}

void SendObstacle(vector<Vector2> obstacle)
{
	int obstaclePointNum = obstacle.size();
	MPI_Bcast(&obstaclePointNum, 1, MPI_INT, 0, MPI_COMM_WORLD);

	for(size_t i = 0; i < obstacle.size(); i++)
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
map<int, pair<Vector2, Vector2>> DivideModelingArea(const pair<Vector2, Vector2> &globalArea, int adjacentAreaWidth)
{
	float minX = globalArea.first.x();
	float minY = globalArea.first.y();
	float maxX = globalArea.second.x();
	float maxY = globalArea.second.y();

	vector<pair<Vector2, Vector2> > resultAreas;
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
					if((resultAreas[i].second.y() - resultAreas[i].first.y()) / 2 < adjacentAreaWidth)
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
					if ((resultAreas[i].second.x() - resultAreas[i].first.x()) / 2 < adjacentAreaWidth)
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

	map<int, pair<Vector2, Vector2>> areas;
	for(int i = 0; i < resultAreas.size(); i++)
	{
		areas[i+1] = resultAreas[i];
	}

	return areas;
}

void SavePartitionedAreasToJSON(map<int, pair<Vector2, Vector2> > modelingAreas , const string path, int adjacentAreaWidth)
{
	std::fstream modelingSubareasFile;
	modelingSubareasFile.open(path.c_str(), ios::out | ios::trunc);
	modelingSubareasFile << "[" << endl;
	//for(int i = 0; i < modelingAreas.size(); i++)
	int i = 0;
	for(map<int, pair<Vector2, Vector2> >::iterator it = modelingAreas.begin(); it != modelingAreas.end(); ++it)
	{
		modelingSubareasFile << "{" << endl;

		modelingSubareasFile << "\"verticalIndent\":" << adjacentAreaWidth << "," << endl;
		modelingSubareasFile << "\"horizontalIndent\":" << adjacentAreaWidth << "," << endl;
		modelingSubareasFile << "\"x1\":" << it->second.first.x() << "," << endl;
		modelingSubareasFile << "\"y1\":" << it->second.first.y() << "," << endl;
		modelingSubareasFile << "\"x2\":" << it->second.second.x() << "," << endl;
		modelingSubareasFile << "\"y2\":" << it->second.second.y() << endl;

		if(i < modelingAreas.size() - 1)
		{
			modelingSubareasFile << "}," << endl;
		}
		else
		{
			modelingSubareasFile << "}" << endl;
		}

		i++;
	}

	modelingSubareasFile << "]" << endl;
	modelingSubareasFile.close();
}

vector<Vector2> ModelingAreaPartitioning(char* argv[])
{
	vector<Vector2> agentsPositions;

	Vector2 p1(atoi(argv[1]), atoi(argv[2])); //Minimal point of modeling area
	Vector2 p2(atoi(argv[3]), atoi(argv[4])); //Maximal point of modeling area

	GlobalArea = CreateModelingArea(obstacles, p1, p2, 1); //Create obstacle around modeling area (rectangle)

	//Agent radius where it interact with anothers
	adjacentAreaWidth = atoi(argv[5]);
	//float minimalHeight = atoi(argv[5]);

	totalAgentsCount =  atoi(argv[6]);

	agentsPositions = GenerateRandomAgentsPositions();

	modelingAreas = DivideModelingArea(GlobalArea, adjacentAreaWidth);

	if(myRank == 0)
	{
		SaveObstaclesToJSON(obstacles, "Obstacles.txt");
		SavePartitionedAreasToJSON(modelingAreas ,"areas.txt", adjacentAreaWidth);
	}

	return agentsPositions;
}

pair<Vector2, Vector2> CreateModelingArea(vector<vector<Vector2> > &obstacles, Vector2 minPoint, Vector2 maxPoint, float borderWidth)
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

	pair<Vector2, Vector2> globalArea(minPoint, maxPoint);
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

void SaveObstaclesToJSON(vector<vector<Vector2> > obstacles, const string path)
{
	std::fstream agentsPositionsFile;
	agentsPositionsFile.open(path.c_str(), ios::out | ios::trunc);
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

void SaveSimDataToFile(string filename, vector<map<long long, pair<Vector2, AgentOnNodeInfo>>> simulationData)
{
	int writingToFileStartTime = clock();
	std::fstream agentsPositionsFile;
	agentsPositionsFile.open("simData.txt", ios::out | ios::trunc);
	agentsPositionsFile << "[" << endl;

	for(int i = 0; i < simulationData.size(); i++)
	{
		agentsPositionsFile << "{\"iteration\": \"" << i << "\"," << endl;

		agentsPositionsFile << "\"agents\": [" << endl;

		bool notFirstAgentInSection = false;
		for (std::map<long long, pair<Vector2, AgentOnNodeInfo>>::iterator agentsIterator = simulationData[i].begin(); agentsIterator !=  simulationData[i].end(); ++agentsIterator)
		{
			if (!agentsIterator->second.second.isDeleted)
			{
				if(notFirstAgentInSection)
				{
					agentsPositionsFile << "," << endl;
				}

				agentsPositionsFile << "{" << endl;
				agentsPositionsFile << "\t\"agentID\" : \"" << agentsIterator->first << "\"," <<endl;
				agentsPositionsFile << "\t\"agentCurrentNode\" : \"" << agentsIterator->second.second._nodeID << "\"," <<endl;
				agentsPositionsFile << "\t\"X\" : \"" << agentsIterator->second.first.x() << "\"," << endl;
				agentsPositionsFile << "\t\"Y\" : \"" << agentsIterator->second.first.y() << "\"" << endl;
				agentsPositionsFile << "}";

				notFirstAgentInSection = true;
			}
		}
		agentsPositionsFile << "\n]" << endl;

		agentsPositionsFile << "}";
		if(i < simulationData.size() - 1)
		{
			agentsPositionsFile << ",";
		}
	}

	agentsPositionsFile << "]" << endl;
	agentsPositionsFile.close();

	printf ("Writing to file time: (%f seconds).\n",((float)clock() - writingToFileStartTime)/CLOCKS_PER_SEC);
}

void AgentPropertyConfigBcasting()
{
	unsigned char* serializedDefaultAgentConfig;
	size_t buffSize = 0;

	if (myRank == 0)
	{
		defaultAgentConfig = new AgentPropertyConfig
			(
			15.f, 	//	NeighborDist = 15f,
			15, 	//	MaxNeighbors = 10,
			5.f, 	//	TimeHorizon = 5f,
			0.2f,	//	Radius = 0.2f,
			2.0f,	//	MaxSpeed = 2.0f,
			1.0f, 	//	float force, was 0
			0.5f,	//	float accelerationCoefficient,
			1.f, 	//	float relaxationTime,
			0.2f,	//	float repulsiveAgent, was 1.2
			70, 	//	float repulsiveAgentFactor, was 70
			0.1f, 	//	float repulsiveObstacle,
			0.3f, 	//	float repulsiveObstacleFactor,
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
		simulator->setAgentDefaults(*defaultAgentConfig);
	}

	delete[] serializedDefaultAgentConfig;
}

vector<Vector2> GenerateRandomAgentsPositions()
{
	vector<Vector2> agentsPositions;

	//Generating agents at main node
	if (myRank == 0)
	{
		int h = GlobalArea.second.y() - GlobalArea.first.y();
		int w = GlobalArea.second.x() - GlobalArea.first.x();
		Vector2 zoneAMinPoint(0, 0);
		Vector2 zoneAMaxPoint(0.3 * w, h);
		pair<Vector2, Vector2> zoneA(zoneAMinPoint, zoneAMaxPoint);

		Vector2 zoneBMinPoint(0.7 * w, 0);
		Vector2 zoneBMaxPoint(w, h);
		pair<Vector2, Vector2> zoneB(zoneBMinPoint, zoneBMaxPoint);

		int agentsInAZone = totalAgentsCount / 2;
		int agentsInBZone = totalAgentsCount / 2;

		//Agents generating zones
		Vector2 zomeAMinPoint = zoneA.first; 
		Vector2 zomeAMaxPoint = zoneA.second;
		Vector2 zomeBMinPoint = zoneB.first; 
		Vector2 zomeBMaxPoint = zoneB.second;

		srand(time(NULL));
		//random agents in zone A
		for (int i = 0; i < agentsInAZone; i++)
		{
			Vector2 agentPosition(GenerateRandomBetween(zomeAMinPoint.x(), zomeAMaxPoint.x()), GenerateRandomBetween(zomeAMinPoint.y(), zomeAMaxPoint.y()));
			agentsPositions.push_back(agentPosition);
		}

		//random agents in zone B
		for (int i = 0; i < agentsInBZone; i++)
		{
			Vector2 agentPosition(GenerateRandomBetween(zomeBMinPoint.x(), zomeBMaxPoint.x()), GenerateRandomBetween(zomeBMinPoint.y(), zomeBMaxPoint.y()));
			agentsPositions.push_back(agentPosition);
		}
	}

	return agentsPositions;
}

void BcastingObstacles()
{
	if(myRank == 0)
	{
		//vector size calculating
		int bufferSize = 0;
		bufferSize += sizeof(int); //number of obstacles
		for(int i = 0; i < obstacles.size(); i++)
		{
			bufferSize += sizeof(int); //number of points
			for(int j = 0; j < obstacles[i].size(); j++)
			{
				bufferSize += 2 * sizeof(float);
			}
		}

		// The buffer we will be writing bytes into
		unsigned char* outBuf = new unsigned char[bufferSize];
		//cout << "Rank: " << myRank << " buffer size: " << bufferSize << endl;

		// A pointer we will advance whenever we write data
		unsigned char* p = outBuf;

		int listSize = obstacles.size();
		//cout << "Rank: " << myRank << " "<< listSize << " objects prepare to sending"<< endl;
		memcpy(p, &listSize, sizeof(int));
		p += sizeof(int);
		//cout << "Rank: " << myRank << " written:  " << p - outBuf << endl;

		for(int i = 0; i < obstacles.size(); i++)
		{
			int pointsNumInObstacle = obstacles[i].size();
			memcpy(p, &pointsNumInObstacle, sizeof(int));
			p += sizeof(int);
			//cout << "Rank: " << myRank << " written:  " << p - outBuf << endl;
			//cout << "Rank: " << myRank << " " << pointsNumInObstacle << " points would be written serialized." << endl;
			for(int j = 0; j < obstacles[i].size(); j++)
			{
				float x = obstacles[i][j].x();
				memcpy(p, &x, sizeof(float));
				p += sizeof(float);
				//cout << "Rank: " << myRank << " written:  " << p - outBuf << endl;
				//cout << "Rank: " << myRank << " point " << x << " serialized." << endl;

				float y = obstacles[i][j].y();
				memcpy(p, &y, sizeof(float));
				p += sizeof(float);
				//cout << "Rank: " << myRank << " written:  " << p - outBuf << endl;
				//cout << "Rank: " << myRank << " point " << y << " serialized." << endl;
			}
		}

		MPI_Bcast(&bufferSize, 1, MPI_INT, 0, MPI_COMM_WORLD);
		//cout << "Rank: " << myRank << " buffer size bcasted: " << bufferSize << endl;
		MPI_Bcast(outBuf, bufferSize, MPI_UNSIGNED_CHAR, 0, MPI_COMM_WORLD);
		//cout << "Rank: " << myRank << " array bcasted: " << bufferSize << endl;
		delete[] outBuf;
		//cout << "Rank: " << myRank << " delete operation performed." << endl;
	}
	else
	{
		int arrayLength;
		MPI_Bcast(&arrayLength, 1, MPI_INT, 0, MPI_COMM_WORLD);
		//cout << "Rank: " << myRank << " array size obtained: " << arrayLength << endl;
		unsigned char* obstacleArray = new unsigned char[arrayLength];
		// A pointer we will advance whenever we write data

		MPI_Bcast(obstacleArray, arrayLength, MPI_UNSIGNED_CHAR, 0, MPI_COMM_WORLD);
		//cout << "Rank: " << myRank << " array obtailned." << endl;

		unsigned char* p = obstacleArray;

		//obstacles deserializing
		int obstaclesNum = 0;
		memcpy(&obstaclesNum, p, sizeof(obstaclesNum));
		p += sizeof(obstaclesNum);
		//cout << "Rank: " << myRank << " in array " << obstaclesNum << " elements" << endl;


		for(int i = 0; i < obstaclesNum; i++)
		{
			int pointsNum = 0;
			memcpy(&pointsNum, p, sizeof(pointsNum));
			p += sizeof(pointsNum);
			//cout << "Rank: " << myRank << " there " << pointsNum << " points" << endl;

			vector<Vector2> obstacle;
			for(int j = 0; j < pointsNum; j++)
			{
				float x = 0;
				memcpy(&x, p, sizeof(x));
				p += sizeof(x);
				//cout << "Rank: " << myRank << " x: " << x << "" << endl;

				float y = 0;
				memcpy(&y, p, sizeof(y));
				p += sizeof(y);
				//cout << "Rank: " << myRank << " y: " << y << "" << endl;
				obstacle.push_back(Vector2(x, y));
			}

			obstacles.push_back(obstacle);
		}

		delete[] obstacleArray;
		//cout << "Rank: " << myRank << " delete operation performed." << endl;

		for(int i = 0; i < obstacles.size(); i++)
		{
			simulator->addObstacle(obstacles[i]);
		}
		simulator->processObstacles();
		//cout << "Rank: " << myRank << " all obstacles proceesed." << endl;
	}
}

void BroadcastingGeneratedAgents(vector<Vector2> agentsPositions)
{
	if(myRank == 0)
	{
		for (int i = 0; i < agentsPositions.size(); i++)
		{
			int destinationNode = 0;
			bool IsPOintAdded = false;
			for(map<int, pair<Vector2, Vector2>>::iterator it = modelingAreas.begin(); it != modelingAreas.end(); ++it)
			{
				if(agentsPositions[i].x() >= it->second.first.x() && agentsPositions[i].x() < it->second.second.x())
				{
					if(agentsPositions[i].y() >= it->second.first.y() && agentsPositions[i].y() < it->second.second.y())
					{
						destinationNode = it->first;
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

						AgentsIDMap[totalAgentsIDs] = AgentOnNodeInfo(destinationNode, newAgentID);
						NodesAgentsMap[destinationNode][newAgentID] = totalAgentsIDs;

						AgentsPositions[totalAgentsIDs] = Vector2(x, y);
						totalAgentsIDs++;
						//cout << "rank: " << myRank << " Position " << x << ":" << y << " Global ID: "<< totalAgentsIDs - 1  << " Node: "<< AgentsIDMap[totalAgentsIDs - 1]._nodeID << " ID: " << AgentsIDMap[totalAgentsIDs - 1]._agentID << endl;
						IsPOintAdded = true;
					}
				}
			}
			if (!IsPOintAdded)
			{
				cout << "rank: " << myRank << " Position: " << agentsPositions[i].x() << " : " << agentsPositions[i].y() << " wasnt added" << endl;
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
				//if(x < )
				size_t agentID = simulator->addAgent(Vector2(x, y));
				//cout << "rank: " << myRank << " new agend added to simulator. ID: " << agentID << endl;
				MPI_Send(&agentID, 1, MPI_INT, 0, 0, MPI_COMM_WORLD); //Sending back new agent ID
				//cout << "rank: " << myRank << " ID of new agent sent to main node." << endl;
			}
		} 
		while (receivedIndex > 0);
	}
}

void SendNewVelocities()
{
	int destinationNode = 0;
	long long agentId = -1;
	float xVel = 0;
	float yVel = 0;
	int agNum = AgentsIDMap.size();

	if(myRank == 0)
	{
		int velocitiesSendingStartTime = clock();

		int h = GlobalArea.second.y() - GlobalArea.first.y();
		int w = GlobalArea.second.x() - GlobalArea.first.x();

		size_t agentWithVelSize = sizeof(long long) + sizeof(float) + sizeof(float);
		//for(int i = 0; i < modelingAreas.size(); i++)
		//for(map<int, pair<Vector2, Vector2>>::iterator it = modelingAreas.begin(); it != modelingAreas.end(); ++it)
		for (map<int, map<long long, long long>>::iterator ndIter = NodesAgentsMap.begin(); ndIter != NodesAgentsMap.end(); ++ndIter)
		{
			vector<long long> agentsToSend;
			int position = 0;
			destinationNode = ndIter->first;
			MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);

			for (map<long long, long long>::iterator idIter = ndIter->second.begin(); idIter != ndIter->second.end(); ++idIter)
			{
				if (!AgentsIDMap[idIter->second].isDeleted)
				{
					agentsToSend.push_back(idIter->second);
				}
			}

			size_t agentsToSendNum = agentsToSend.size();
			size_t buffSize = 0;
			buffSize = agentWithVelSize * agentsToSendNum + sizeof(int);
			unsigned char* buffer = new unsigned char[buffSize];

			MPI_Pack(&agentsToSendNum, 1, MPI_INT, buffer, buffSize, &position, MPI_COMM_WORLD);

			for (int ag = 0; ag < agentsToSend.size(); ag++) //packing agents data
			{
				agentId = AgentsIDMap[agentsToSend[ag]]._agentID;

				if (agentsToSend[ag] < agNum / 2) //Agents from zone A NOT WORKING IN COMMON CASE
				{
					if (AgentsPositions[agentsToSend[ag]].x() <= 0.98 * w)
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
					if (AgentsPositions[agentsToSend[ag]].x() >= 0.02 * w)
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

				//cout << "Agent packing ID: " << agentId << " xvel: " << xVel << " yVel: " << yVel << endl; 
				MPI_Pack(&agentId, 1, MPI_LONG_LONG_INT, buffer, buffSize, &position, MPI_COMM_WORLD);
				MPI_Pack(&xVel, 1, MPI_FLOAT, buffer, buffSize, &position, MPI_COMM_WORLD);
				MPI_Pack(&yVel, 1, MPI_FLOAT, buffer, buffSize, &position, MPI_COMM_WORLD);
			}

			MPI_Send(&buffSize, 1, MPI_INT, destinationNode, 100, MPI_COMM_WORLD);
			MPI_Send(buffer, position, MPI_PACKED, destinationNode, 100, MPI_COMM_WORLD);
			delete[] buffer;
		}

		//End of broadcasting flag
		destinationNode = -1;
		MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
		printf("Velocities sending time: (%f seconds).\n", ((float)clock() - velocitiesSendingStartTime) / CLOCKS_PER_SEC);
	}
	else
	{
		if(myRank < modelingAreas.size() + 1)
		{
			//cout << "node: " << myRank << " receiving velocities started " << endl;
			do
			{
				int buffSize = 0;
				MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD); //Check if it information about agent on this node
				//cout << "Node " << destinationNode << " receive agents new velocities" << endl;
				if (destinationNode == myRank)
				{
					int position = 0;
					MPI_Recv(&buffSize, 1, MPI_INT, 0, 100, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					//cout << "Buffer size " << buffSize << endl;
					unsigned char* buffer = new unsigned char[buffSize];
					MPI_Recv(buffer, buffSize, MPI_PACKED, 0, 100, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					//cout << "Buffer received " << buffSize << endl;
					int agentsCount = 0;
					MPI_Unpack(buffer, buffSize, &position, &agentsCount, 1, MPI_INT, MPI_COMM_WORLD);
					//cout << "In a buffer agents count " << agentsCount << endl;
					for(int i = 0; i < agentsCount; i++)
					{
						MPI_Unpack(buffer, buffSize, &position, &agentId, 1, MPI_LONG_LONG_INT, MPI_COMM_WORLD);
						MPI_Unpack(buffer, buffSize, &position, &xVel, 1, MPI_FLOAT, MPI_COMM_WORLD);
						MPI_Unpack(buffer, buffSize, &position, &yVel, 1, MPI_FLOAT, MPI_COMM_WORLD);
						//cout << "Agent unpacked ID: " << agentId << " xvel: " << xVel << " yVel: " << yVel << endl;
						simulator->setAgentPrefVelocity(agentId, Vector2(xVel, yVel));
					}
					//cout << "All received agents added to model" << endl;
					delete[] buffer;
				}
			} while (destinationNode > 0);
			//cout << "node: " << myRank << " receiving velocities finished " << endl;
		}
	}
}

void ExchangingByPhantoms()
{
	int ExchangingByPhantomsStartTime = clock();
	map<int, vector<unsigned char*>> agentsToShift;
	for(int i = 1; i < modelingAreas.size() + 1; i++) //Initializing map
	{
		vector<unsigned char*> serializedAgentsVector;
		agentsToShift[i] = serializedAgentsVector; 
	}

	if(myRank < modelingAreas.size() + 1 && myRank != 0)
	{
		float x;
		float y;
		vector<size_t> aliveAgents = simulator->getAliveAgentIdsList();
		for(int i = 0; i < aliveAgents.size(); i++)
		{
			MPIAgent agent = MPIAgent(simulator->getAgent(aliveAgents[i]));
			x = agent.Position().x();
			y = agent.Position().y();

			pair<Vector2, Vector2> myArea = modelingAreas[myRank];

			if(		x >= myArea.first.x()	&& x <= myArea.first.x() + adjacentAreaWidth 
				||	x <= myArea.second.x()	&& x >= myArea.second.x() - adjacentAreaWidth
				||	y >= myArea.first.y()	&& y <= myArea.first.y() + adjacentAreaWidth 
				||	y <= myArea.second.y()	&& y >= myArea.second.y() - adjacentAreaWidth ) //checking for placing in adjacent area
			{
				for(map<int, pair<Vector2, Vector2>>::iterator ar = modelingAreas.begin(); ar != modelingAreas.end(); ++ar)
				{
					if(ar->first != myRank)
					{
#pragma region left side
						if(ar->second.second.y() == modelingAreas[myRank].second.y()
							&& ar->second.first.y() == modelingAreas[myRank].first.y()
							&& ar->second.second.x() == modelingAreas[myRank].first.x())
						{
							unsigned char* serializedAgent = agent.SerializeAgent();
							agentsToShift[ar->first].push_back(serializedAgent);
						}
#pragma endregion left side

#pragma region right side
						if(modelingAreas[myRank].second.y() == ar->second.second.y()//if common side
							&& modelingAreas[myRank].second.x() == ar->second.first.x()
							&& modelingAreas[myRank].first.y() == ar->second.first.y())
						{
							unsigned char* serializedAgent = agent.SerializeAgent();
							agentsToShift[ar->first].push_back(serializedAgent);
						}
#pragma endregion right side

#pragma region bottom side
						if(modelingAreas[myRank].first.x() == ar->second.first.x()//if common side
							&& modelingAreas[myRank].first.y() == ar->second.second.y()
							&& modelingAreas[myRank].second.x() == ar->second.second.x())
						{
							unsigned char* serializedAgent = agent.SerializeAgent();
							agentsToShift[ar->first].push_back(serializedAgent);
						}
#pragma endregion bottom side

#pragma region top side
						if(	ar->second.first.x() == modelingAreas[myRank].first.x()//if common side
							&& ar->second.first.y() == modelingAreas[myRank].second.y()
							&& ar->second.second.x() == modelingAreas[myRank].second.x())
						{
							unsigned char* serializedAgent = agent.SerializeAgent();
							agentsToShift[ar->first].push_back(serializedAgent);
						}
#pragma endregion top side

						//diagonals
#pragma region top right corner
						if(ar->second.first.x() == modelingAreas[myRank].second.x()//if common side
							&& ar->second.first.y() == modelingAreas[myRank].second.y())
						{
							unsigned char* serializedAgent = agent.SerializeAgent();
							agentsToShift[ar->first].push_back(serializedAgent);
						}
#pragma endregion top right corner

#pragma region top left corner
						if(ar->second.second.x() == modelingAreas[myRank].first.x()//if common side
							&& ar->second.first.y() == modelingAreas[myRank].second.y())
						{
							unsigned char* serializedAgent = agent.SerializeAgent();
							agentsToShift[ar->first].push_back(serializedAgent);
						}
#pragma endregion top left corner

#pragma region bottom left corner
						if(ar->second.second.x() == modelingAreas[myRank].first.x()//if common side
							&& ar->second.second.y() == modelingAreas[myRank].first.y())
						{
							unsigned char* serializedAgent = agent.SerializeAgent();
							agentsToShift[ar->first].push_back(serializedAgent);
						}
#pragma endregion bottom left corner

#pragma region bottom right corner
						if(ar->second.first.x() == modelingAreas[myRank].second.x()//if common side
							&& ar->second.second.y() == modelingAreas[myRank].first.y())
						{
							unsigned char* serializedAgent = agent.SerializeAgent();
							agentsToShift[ar->first].push_back(serializedAgent);
						}
#pragma endregion bottom right corner
					}
				}
			}
		}

		int buffSize = 0;
		unsigned char* buffer;
		MPI_Status stat;
		vector<unsigned char*> receivedAgents;
		int position = 0;
		int agentSize = 0;

		for(map<int, pair<Vector2, Vector2>>::iterator nd = modelingAreas.begin(); nd != modelingAreas.end(); ++nd)
		{
			//cout << "Current shifting iter: " << nd->first << endl;
			buffSize = 0;
			if(myRank == nd->first) //Receiving
			{
				for(int senderNodes = 0; senderNodes < modelingAreas.size() - 1; senderNodes++ )
				{
					MPI_Recv(&buffSize, 1, MPI_INT, MPI_ANY_SOURCE, 100, MPI_COMM_WORLD, &stat);
					int source = stat.MPI_SOURCE;
					//cout << myRank << " In receiving from: " << source << " Iter: " << senderNodes << " Buffer size: " << buffSize << endl;
					if(buffSize > 0)
					{
						position = 0;
						buffer = new unsigned char[buffSize];
						MPI_Recv(buffer, buffSize, MPI_UNSIGNED_CHAR, source, 100, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
						//cout << " Buffer received" << endl;

						//unpacking and puting to receivedAgents
						do
						{
							agentSize = 0;
							MPI_Unpack(buffer, buffSize, &position, &agentSize, 1, MPI_INT, MPI_COMM_WORLD);
							unsigned char* serializedAgent = new unsigned char[agentSize];
							MPI_Unpack(buffer, buffSize, &position, serializedAgent, agentSize, MPI_UNSIGNED_CHAR, MPI_COMM_WORLD);
							receivedAgents.push_back(serializedAgent);
							//cout << " Agent unpacked: " << position << " " << buffSize << endl;
						}
						while (position < buffSize);
						delete[] buffer;
					}
				}
			}
			else //Sending
			{
				buffSize = 0;
				//cout << myRank << " Im sending to" << nd->first << endl;
				if(agentsToShift[nd->first].size() > 0)
				{
					for(int i = 0; i < agentsToShift[nd->first].size(); i++)
					{
						agentSize = 0;
						memcpy(&agentSize, agentsToShift[nd->first][i], sizeof(int));
						buffSize+= sizeof(int) + agentSize;
					}
					//cout << myRank << " I have to send " << agentsToShift[nd->first].size() << " agents with size: " << buffSize << endl;

					buffer = new unsigned char[buffSize];
					position = 0;
					for(int i = 0; i < agentsToShift[nd->first].size(); i++)
					{
						agentSize = 0;
						memcpy(&agentSize, agentsToShift[nd->first][i], sizeof(int));
						MPI_Pack(&agentSize, 1, MPI_INT, buffer, buffSize, &position, MPI_COMM_WORLD);
						MPI_Pack(agentsToShift[nd->first][i], agentSize, MPI_UNSIGNED_CHAR, buffer, buffSize, &position, MPI_COMM_WORLD);
					}
					//cout << myRank << " All data packed " << endl;

					MPI_Send(&buffSize, 1, MPI_INT, nd->first, 100, MPI_COMM_WORLD);
					MPI_Send(buffer, position, MPI_PACKED, nd->first, 100, MPI_COMM_WORLD);
					//cout << myRank << " All data sent " << endl;
					delete[] buffer;
				}
				else
				{
					MPI_Send(&buffSize, 1, MPI_INT, nd->first, 100, MPI_COMM_WORLD);
					//cout << myRank << " Zero sent " << endl;
				}
			}
		}

		for(int i = 0; i < receivedAgents.size(); i++)
		{
			Agent* tmpAgent = Agent::Deseriaize(receivedAgents[i]);
			simulator->addTempAgent(tmpAgent);
			delete[] receivedAgents[i];
		}
		receivedAgents.clear();

		printf ("ExchangingByPhantoms time: (%f seconds).\n",((float)clock() - ExchangingByPhantomsStartTime)/CLOCKS_PER_SEC);
	}

	//	#pragma region OLD EXCHANGING ALGORITHM
	//	if(myRank == 0)
	//	{
	//		int phantomCheckStart = clock();
	//		int destinationNode = 0;
	//		int phantomsNum = 0;
	//		for (std::map<long long, AgentOnNodeInfo>::iterator it= AgentsIDMap.begin(); it != AgentsIDMap.end(); ++it)
	//		{
	//			if(!it->second.isDeleted)
	//			{
	//				float x = AgentsPositions[it->first].x();
	//				float y = AgentsPositions[it->first].y();
	//				//cout << "agent pos x: " << x << " y: " << y << endl;
	//
	//#pragma region left side
	//				if( x <= modelingAreas[it->second._nodeID].first.x() + adjacentAreaWidth) // left side adjacent area
	//				{
	//					//for(int i = 0; i < modelingAreas.size(); i++)
	//					for(map<int, pair<Vector2, Vector2>>::iterator ar = modelingAreas.begin(); ar != modelingAreas.end(); ++ar)
	//					{
	//						if (it->second._nodeID != ar->first)
	//						{
	//							if(ar->second.second.y() == modelingAreas[it->second._nodeID].second.y()//if common side
	//								&& ar->second.first.y() == modelingAreas[it->second._nodeID].first.y()
	//								&& ar->second.second.x() == modelingAreas[it->second._nodeID].first.x())
	//							{
	//								//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by left side" << endl;
	//								//request agent
	//								destinationNode = it->second._nodeID;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								long long nodeAgentId = it->second._agentID;
	//								MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID , 0, MPI_COMM_WORLD);
	//								//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
	//								//receiving agent
	//								int SerializedAgentSize = 0;
	//								MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "agert size received" << endl;
	//								unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
	//								MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "Agent received" << endl;
	//
	//								//send agent to create phantom
	//								destinationNode = ar->first;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								//cout << "Bcasted destination node" << endl;
	//								//sending agent
	//								MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
	//								MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
	//								delete[] serializedAgent;
	//
	//								phantomsNum++;
	//								//cout << "Phantom sent to the left " << it->first << " from: " << it->second._nodeID << " to: "<< destinationNode << endl;
	//							}
	//						}
	//					}
	//				}
	//#pragma endregion left side
	//
	//#pragma region right side
	//				if(x >= modelingAreas[it->second._nodeID].second.x() - adjacentAreaWidth) // right side adjacent area
	//				{
	//					//for(int i = 0; i < modelingAreas.size(); i++)
	//					for(map<int, pair<Vector2, Vector2>>::iterator ar = modelingAreas.begin(); ar != modelingAreas.end(); ++ar)
	//					{
	//						if (it->second._nodeID != ar->first)
	//						{
	//							if(modelingAreas[it->second._nodeID].second.y() == ar->second.second.y()//if common side
	//								&& modelingAreas[it->second._nodeID].second.x() == ar->second.first.x()
	//								&& modelingAreas[it->second._nodeID].first.y() == ar->second.first.y())
	//							{
	//								//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by right side" << endl;
	//								//request agent
	//								destinationNode = it->second._nodeID;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								long long nodeAgentId = it->second._agentID;
	//								MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID ,0, MPI_COMM_WORLD);
	//								//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
	//								//receiving agent
	//								int SerializedAgentSize = 0;
	//								MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "agert size received" << endl;
	//								unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
	//								MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "Agent received" << endl;
	//
	//								//send agent to create phantom
	//								destinationNode = ar->first;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								//cout << "Bcasted destination node" << endl;
	//								//sending agent
	//								MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
	//								MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
	//								delete[] serializedAgent;
	//
	//								phantomsNum++;
	//								//cout << "Phantom sent to the right " << it->first <<  " from: " << it->second._nodeID << " to: "<< destinationNode << endl;
	//							}
	//						}
	//					}
	//				}
	//#pragma endregion right side
	//
	//#pragma region bottom side
	//				if(y <= modelingAreas[it->second._nodeID].first.y() + adjacentAreaWidth) // bottom side adjacent area
	//				{
	//					//for(int i = 0; i < modelingAreas.size(); i++)
	//					for(map<int, pair<Vector2, Vector2>>::iterator ar = modelingAreas.begin(); ar != modelingAreas.end(); ++ar)
	//					{
	//						if (it->second._nodeID != ar->first)
	//						{
	//							if(modelingAreas[it->second._nodeID].first.x() == ar->second.first.x()//if common side
	//								&& modelingAreas[it->second._nodeID].first.y() == ar->second.second.y()
	//								&& modelingAreas[it->second._nodeID].second.x() == ar->second.second.x())
	//							{
	//								//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by bottom side" << endl;
	//								//request agent
	//								destinationNode = it->second._nodeID;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								long long nodeAgentId = it->second._agentID;
	//								MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID ,0, MPI_COMM_WORLD);
	//								//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
	//								//receiving agent
	//								int SerializedAgentSize = 0;
	//								MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "agert size received" << endl;
	//								unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
	//								MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "Agent received" << endl;
	//
	//								//send agent to create phantom
	//								destinationNode = ar->first;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								//cout << "Bcasted destination node" << endl;
	//								//sending agent
	//								MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
	//								MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
	//								delete[] serializedAgent;
	//
	//								phantomsNum++;
	//								//cout << "Phantom sent to the bottom " << it->first <<  " from: " << it->second._nodeID << " to: "<< destinationNode << endl;
	//							}
	//						}
	//					}
	//				}
	//#pragma endregion bottom side
	//
	//#pragma region top side
	//				if(y >= modelingAreas[it->second._nodeID].second.y() - adjacentAreaWidth) // top side adjacent area
	//				{
	//					//for(int i = 0; i < modelingAreas.size(); i++)
	//					for(map<int, pair<Vector2, Vector2>>::iterator ar = modelingAreas.begin(); ar != modelingAreas.end(); ++ar)
	//					{
	//						if (it->second._nodeID != ar->first)
	//						{
	//							if(ar->second.first.x() == modelingAreas[it->second._nodeID].first.x()//if common side
	//								&& ar->second.first.y() == modelingAreas[it->second._nodeID].second.y()
	//								&& ar->second.second.x() == modelingAreas[it->second._nodeID].second.x())
	//							{
	//								//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by top side" << endl;
	//								//request agent
	//								destinationNode = it->second._nodeID;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								long long nodeAgentId = it->second._agentID;
	//								MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID , 0, MPI_COMM_WORLD);
	//								//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
	//								//receiving agent
	//								int SerializedAgentSize = 0;
	//								MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "agert size received" << endl;
	//								unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
	//								MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "Agent received" << endl;
	//
	//								//send agent to create phantom
	//								destinationNode = ar->first;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								//cout << "Bcasted destination node" << endl;
	//								//sending agent
	//								MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
	//								MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
	//								delete[] serializedAgent;
	//
	//								phantomsNum++;
	//								//cout << "Phantom sent to the top " << it->first <<  " from: " << it->second._nodeID << " to: "<< destinationNode << endl;
	//							}
	//						}
	//					}
	//				}
	//#pragma endregion top side
	//
	//				//diagonals
	//#pragma region top right corner
	//				if(y >= modelingAreas[it->second._nodeID].second.y() - adjacentAreaWidth 
	//					&& x >= modelingAreas[it->second._nodeID].second.x() - adjacentAreaWidth) // top side adjacent area
	//				{
	//					//for(int i = 0; i < modelingAreas.size(); i++)
	//					for(map<int, pair<Vector2, Vector2>>::iterator ar = modelingAreas.begin(); ar != modelingAreas.end(); ++ar)
	//					{
	//						if (it->second._nodeID != ar->first)
	//						{
	//							if(ar->second.first.x() == modelingAreas[it->second._nodeID].second.x()//if common side
	//								&& ar->second.first.y() == modelingAreas[it->second._nodeID].second.y())
	//							{
	//								//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by top side" << endl;
	//								//request agent
	//								destinationNode = it->second._nodeID;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								long long nodeAgentId = it->second._agentID;
	//								MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID , 0, MPI_COMM_WORLD);
	//								//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
	//								//receiving agent
	//								int SerializedAgentSize = 0;
	//								MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "agert size received" << endl;
	//								unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
	//								MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "Agent received" << endl;
	//
	//								//send agent to create phantom
	//								destinationNode = ar->first;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								//cout << "Bcasted destination node" << endl;
	//								//sending agent
	//								MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
	//								MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
	//								delete[] serializedAgent;
	//
	//								phantomsNum++;
	//								//cout << "Phantom sent to the top right " << it->first <<  " from: " << it->second._nodeID << " to: "<< destinationNode << endl;
	//							}
	//						}
	//					}
	//				}
	//#pragma endregion top right corner
	//
	//#pragma region top left corner
	//				if(y >= modelingAreas[it->second._nodeID].second.y() - adjacentAreaWidth 
	//					&& x <= modelingAreas[it->second._nodeID].first.x() + adjacentAreaWidth) // top side adjacent area
	//				{
	//					//for(int i = 0; i < modelingAreas.size(); i++)
	//					for(map<int, pair<Vector2, Vector2>>::iterator ar = modelingAreas.begin(); ar != modelingAreas.end(); ++ar)
	//					{
	//						if (it->second._nodeID != ar->first)
	//						{
	//							if(ar->second.second.x() == modelingAreas[it->second._nodeID].first.x()//if common side
	//								&& ar->second.first.y() == modelingAreas[it->second._nodeID].second.y())
	//							{
	//								//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by top side" << endl;
	//								//request agent
	//								destinationNode = it->second._nodeID;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								long long nodeAgentId = it->second._agentID;
	//								MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID , 0, MPI_COMM_WORLD);
	//								//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
	//								//receiving agent
	//								int SerializedAgentSize = 0;
	//								MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "agert size received" << endl;
	//								unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
	//								MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "Agent received" << endl;
	//
	//								//send agent to create phantom
	//								destinationNode = ar->first;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								//cout << "Bcasted destination node" << endl;
	//								//sending agent
	//								MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
	//								MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
	//								delete[] serializedAgent;
	//
	//								phantomsNum++;
	//								//cout << "Phantom sent to the top left " << it->first <<  " from: " << it->second._nodeID << " to: "<< destinationNode << endl;
	//							}
	//						}
	//					}
	//				}
	//#pragma endregion top left corner
	//
	//#pragma region bottom left corner
	//				if(y <= modelingAreas[it->second._nodeID].first.y() + adjacentAreaWidth 
	//					&& x <= modelingAreas[it->second._nodeID].first.x() + adjacentAreaWidth) // top side adjacent area
	//				{
	//					//for(int i = 0; i < modelingAreas.size(); i++)
	//					for(map<int, pair<Vector2, Vector2>>::iterator ar = modelingAreas.begin(); ar != modelingAreas.end(); ++ar)
	//					{
	//						if (it->second._nodeID != ar->first)
	//						{
	//							if(ar->second.second.x() == modelingAreas[it->second._nodeID].first.x()//if common side
	//								&& ar->second.second.y() == modelingAreas[it->second._nodeID].first.y())
	//							{
	//								//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by top side" << endl;
	//								//request agent
	//								destinationNode = it->second._nodeID;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								long long nodeAgentId = it->second._agentID;
	//								MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID , 0, MPI_COMM_WORLD);
	//								//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
	//								//receiving agent
	//								int SerializedAgentSize = 0;
	//								MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "agert size received" << endl;
	//								unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
	//								MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "Agent received" << endl;
	//
	//								//send agent to create phantom
	//								destinationNode = ar->first;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								//cout << "Bcasted destination node" << endl;
	//								//sending agent
	//								MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
	//								MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
	//								delete[] serializedAgent;
	//
	//								phantomsNum++;
	//								//cout << "Phantom sent to the bottom left " << it->first <<  " from: " << it->second._nodeID << " to: "<< destinationNode << endl;
	//							}
	//						}
	//					}
	//				}
	//#pragma endregion bottom left corner
	//
	//#pragma region bottom right corner
	//				if(y <= modelingAreas[it->second._nodeID].first.y() + adjacentAreaWidth 
	//					&& x >= modelingAreas[it->second._nodeID].second.x() - adjacentAreaWidth) // top side adjacent area
	//				{
	//					//for(int i = 0; i < modelingAreas.size(); i++)
	//					for(map<int, pair<Vector2, Vector2>>::iterator ar = modelingAreas.begin(); ar != modelingAreas.end(); ++ar)
	//					{
	//						if (it->second._nodeID != ar->first)
	//						{
	//							if(ar->second.first.x() == modelingAreas[it->second._nodeID].second.x()//if common side
	//								&& ar->second.second.y() == modelingAreas[it->second._nodeID].first.y())
	//							{
	//								//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by top side" << endl;
	//								//request agent
	//								destinationNode = it->second._nodeID;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								long long nodeAgentId = it->second._agentID;
	//								MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID , 0, MPI_COMM_WORLD);
	//								//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
	//								//receiving agent
	//								int SerializedAgentSize = 0;
	//								MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "agert size received" << endl;
	//								unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
	//								MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//								//cout << "Agent received" << endl;
	//
	//								//send agent to create phantom
	//								destinationNode = ar->first;
	//								MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//								//cout << "Bcasted destination node" << endl;
	//								//sending agent
	//								MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
	//								MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
	//								delete[] serializedAgent;
	//
	//								phantomsNum++;
	//								//cout << "Phantom sent to the bottom right " << it->first <<  " from: " << it->second._nodeID << " to: "<< destinationNode << endl;
	//							}
	//						}
	//					}
	//				}
	//#pragma endregion bottom right corner
	//			}
	//		}
	//
	//		//cout << "Total phantomsNum sent: " << phantomsNum << endl;
	//
	//		destinationNode = -1;
	//		MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//		printf ("Phantoms checing and sending time: (%f seconds).\n",((float)clock() - phantomCheckStart)/CLOCKS_PER_SEC);
	//	}
	//	else
	//	{
	//		if(myRank < modelingAreas.size() + 1)
	//		{
	//			int destination = 0;
	//			//cout << "node: " << myRank << " receiving agents from adjacent areas started " << endl;
	//			bool ImReceivingPhantom = false;
	//			size_t receivedPhantomsSize = 0;
	//			do
	//			{
	//				//cout << "waiting bcast" << endl;
	//				MPI_Bcast(&destination, 1, MPI_INT, 0, MPI_COMM_WORLD);
	//				//cout << "Received destination: " << destination << endl;
	//				if(destination == myRank)
	//				{
	//					if(!ImReceivingPhantom)
	//					{
	//						//cout << "Sending phantom" << endl;
	//						long long agentIDtoSend = 0;
	//						MPI_Recv(&agentIDtoSend, 1, MPI_UNSIGNED_LONG_LONG, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//						//cout << "Rank: " << myRank << " received ID:" << agentIDtoSend << endl;
	//						//Send agent to create phantom on adjacent node
	//						unsigned char* agentBuffer = simulator->getAgent(agentIDtoSend)->Serialize();
	//						int SerializedAgentSize = 0;
	//						memcpy(&SerializedAgentSize, agentBuffer, sizeof(int));
	//						//cout << "Rank: " << myRank << " serialized agent size:" << SerializedAgentSize << endl;
	//						MPI_Send(&SerializedAgentSize, 1, MPI_INT, 0, 0, MPI_COMM_WORLD);
	//						MPI_Send(agentBuffer, SerializedAgentSize, MPI_UNSIGNED_CHAR, 0, 0, MPI_COMM_WORLD);
	//						//cout << "Rank: " << myRank << " serialized agent to the main node sent" << endl;
	//						delete[] agentBuffer;
	//					}
	//					else
	//					{
	//						//receiving phantom
	//						//cout << "Receiving a phantom: " << endl;
	//						int SerializedAgentSize = 0;
	//						MPI_Recv(&SerializedAgentSize, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//						//cout << "Rank: " << myRank << " receiving agent with size: " << SerializedAgentSize << endl;
	//						unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
	//						MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
	//						//cout << "Rank: " << myRank << " agent received " << SerializedAgentSize << endl;
	//						receivedPhantomsSize+= SerializedAgentSize;
	//						Agent* tmpAgent = Agent::Deseriaize(serializedAgent);
	//						simulator->addTempAgent(tmpAgent);
	//						//cout << "Rank: " << myRank << " phantom added to a model" << endl;
	//						delete[] serializedAgent;
	//					}
	//				}
	//				ImReceivingPhantom = !ImReceivingPhantom;
	//			}
	//			while (destination > 0);
	//		}
	//	}
	//#pragma endregion OLD EXCHANGING ALGORITHM
}

void UpdateAgentsPositionOnMainNode()
{
	size_t agentId;
	if(myRank == 0)
	{
		int requestNewPositionsStartTime = clock();
		//cout << "modelingAreas.size(): " << modelingAreas.size() << endl;
		MPI_Status stat;
		int agentPosSize = sizeof(size_t) + sizeof(float) + sizeof(float);
		for (int areas = 0; areas < modelingAreas.size(); areas++)
		{
			int agentPOsitionsReceivingStartMoment = clock();
			int agentsPositionsDataReceivingStartTime = clock();
			//cout << "Receiving for area " << areas << endl;
			int agentPosBuffSize;
			int senderNode;
			MPI_Recv(&agentPosBuffSize, 1, MPI_INT, MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &stat);
			//cout << "Receiving from " << stat.MPI_SOURCE << endl;
			if(agentPosBuffSize > 0)
			{
				agentsPositionsDataReceivingStartTime = clock();
				senderNode = stat.MPI_SOURCE;
				unsigned char* agentsPositionsBuffer = new unsigned char[agentPosBuffSize];
				MPI_Recv(agentsPositionsBuffer, agentPosBuffSize, MPI_UNSIGNED_CHAR, senderNode, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
				int agentPositionsNum = agentPosBuffSize / agentPosSize;
				int position = 0;
				//cout << "Agents pos num: " << agentPositionsNum << endl;
				for(int agPos = 0; agPos < agentPositionsNum; agPos++)
				{
					float xPos, yPos;
					MPI_Unpack(agentsPositionsBuffer, agentPosBuffSize, &position, &agentId, 1, MPI_LONG_LONG_INT, MPI_COMM_WORLD);
					MPI_Unpack(agentsPositionsBuffer, agentPosBuffSize, &position, &xPos, 1, MPI_FLOAT, MPI_COMM_WORLD);
					MPI_Unpack(agentsPositionsBuffer, agentPosBuffSize, &position, &yPos, 1, MPI_FLOAT, MPI_COMM_WORLD);

					//cout << "Agents ID: " << agentId << " xPos: " << xPos << " yPos: " << yPos << endl;

					size_t agGlobalId = NodesAgentsMap[senderNode][agentId];
					AgentsPositions[agGlobalId] = Vector2(xPos, yPos);
				}

				delete[] agentsPositionsBuffer;
						printf ("Data requestung time: (%f seconds).\n",((float)clock() - agentsPositionsDataReceivingStartTime)/CLOCKS_PER_SEC);
			}

			printf ("Waiting time: (%f seconds).\n", (float)(agentsPositionsDataReceivingStartTime - agentPOsitionsReceivingStartMoment)/CLOCKS_PER_SEC);
		}
		printf ("Requesting new positions time: (%f seconds).\n",((float)clock() - requestNewPositionsStartTime)/CLOCKS_PER_SEC);
	}
	else
	{
		if(myRank < modelingAreas.size() + 1)
		{
			int agentsNewPositionsStartMoment = clock();

			//Requesting vector with requesting agents IDs
			int agentPosSize = sizeof(size_t) + sizeof(float) + sizeof(float);
			int sizeOfAgentPosBuff = 0;
			//vector<size_t> agentsIds = simulator->getAliveAgentIdsList();
			vector<Agent*> aliveAagents = simulator->getAliveAgents();
			if(aliveAagents.size() > 0)
			{
				//cout << myRank << " I have " << agentsIds.size() << " agents" << endl;
				int AgentPosBuffSize = agentPosSize * aliveAagents.size();
				unsigned char* AgentPosBuffer = new unsigned char[AgentPosBuffSize];
				int position = 0;

				printf ("Alive agents requesting time: (%f seconds).\n",((float)clock() - agentsNewPositionsStartMoment)/CLOCKS_PER_SEC);

				agentsNewPositionsStartMoment = clock();
				MPIAgent agent;
				for(int ag = 0; ag < aliveAagents.size(); ag++)
				{
					//Agent* tmpAgent = simulator->getAgent(agentsIds[ag]);
					agent.agent = aliveAagents[ag]; // simulator->getAgent(agentsIds[ag]);
					agentId = agent.ID();
					float xPos = agent.Position().x();
					float yPos = agent.Position().y();

					//cout << "AgID: " << agid << " xPos: " << xPos << " yPos: " << yPos << endl;
					MPI_Pack(&agentId, 1, MPI_LONG_LONG_INT, AgentPosBuffer, AgentPosBuffSize, &position, MPI_COMM_WORLD);
					MPI_Pack(&xPos, 1, MPI_FLOAT, AgentPosBuffer, AgentPosBuffSize, &position, MPI_COMM_WORLD);
					MPI_Pack(&yPos, 1, MPI_FLOAT, AgentPosBuffer, AgentPosBuffSize, &position, MPI_COMM_WORLD);
				}
				
				printf ("Agents positions packing time: (%f seconds).\n",((float)clock() - agentsNewPositionsStartMoment)/CLOCKS_PER_SEC);
				agentsNewPositionsStartMoment = clock();
				MPI_Send(&AgentPosBuffSize, 1, MPI_INT, 0, 0, MPI_COMM_WORLD);
				//cout << "AgentPosBuffSize: " << AgentPosBuffSize << endl;
				MPI_Send(AgentPosBuffer, position, MPI_PACKED, 0, 0, MPI_COMM_WORLD);
				//cout << "Buffer sent " << endl;
				delete[] AgentPosBuffer;
				printf ("Agents positions sending time: (%f seconds).\n",((float)clock() - agentsNewPositionsStartMoment)/CLOCKS_PER_SEC);
			}
			else
			{ 
				//cout << myRank << " I dont have any agents " << endl;
				MPI_Send(&sizeOfAgentPosBuff, 1, MPI_INT, 0, 0, MPI_COMM_WORLD);
			}
		}
	}
}

void DoSimulationStep()
{
	if(myRank != 0 && myRank < modelingAreas.size() + 1)
	{
		//cout << "node: " << myRank << " simulation step started finished " << endl;
		int sterTimeStart = clock();
		simulator->doStep();
		printf ("%d rank - Step time: (%f seconds).\n", myRank,((float)clock() - sterTimeStart)/CLOCKS_PER_SEC);
	}
}

void AgentsShifting()
{
	int destinationNode = 0;
	if(myRank == 0)
	{
		int agentsSHiftingTimeStart = clock();
		map<long long, MPIAgent> agentsToShift;
		//cout << "agents shifting started" << endl;
		//cout << "Rank: " << myRank << " Sending agents IDs for deleting from simulators"<< endl;
		for (std::map<long long, AgentOnNodeInfo>::iterator it= AgentsIDMap.begin(); it != AgentsIDMap.end(); ++it)
		{
			if(!it->second.isDeleted)
			{
				float x = AgentsPositions[it->first].x();
				float y = AgentsPositions[it->first].y();
				int node = it->second._nodeID;
				if(x < modelingAreas[node].first.x() || modelingAreas[node].second.x() < x || //outside of its modeling area
					y < modelingAreas[node].first.y() || modelingAreas[node].second.y() < y)
				{
					//cout << "gloablID: " << it->first << " localID: " << it->second._agentID <<   " Node:" << node << endl;
					//cout << "agent in coords: " << x << " " << y << " not inside an area: x:" << modelingAreas[node].first.x() << " y:" << modelingAreas[node].first.y() << " x:" << modelingAreas[node].second.x() << " y:" << modelingAreas[node].second.y() << endl; 
					int destination = it->second._nodeID;
					MPI_Bcast(&destination, 1, MPI_INT, 0, MPI_COMM_WORLD);
					//cout << "Rank: " << myRank << " Bcasted destination node: " << destination << endl;
					MPI_Send(&it->second._agentID, 1, MPI_UNSIGNED_LONG_LONG, destination, 0, MPI_COMM_WORLD);
					//cout << "Rank: " << myRank << " ID sended: " << it->second._agentID << endl;
					int serializedAgentSize = 0;
					MPI_Recv(&serializedAgentSize, 1, MPI_INT, destination, 0, MPI_COMM_WORLD, MPI_STATUSES_IGNORE);
					//cout << "Rank: " << myRank << " Receiving serialized agent with size: " << serializedAgentSize << endl;
					unsigned char* serializedAgent = new unsigned char[serializedAgentSize];
					MPI_Recv(serializedAgent, serializedAgentSize, MPI_UNSIGNED_CHAR, destination, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					//cout << "Rank: " << myRank << " Agent received " << endl;
					agentsToShift[it->first] = MPIAgent(Agent::Deseriaize(serializedAgent));
					delete[] serializedAgent;
				}
			}
		}

		//cout << "agents to shift: " << agentsToShift.size() << endl;

		//cout << "Rank: " << myRank << " Bcasting finish flag: " << endl;
		destinationNode = -1;
		MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);  //End of agents shifting requests
		//cout << "Receiving agents to remove finished " << endl;

		//cout << "Sending agents to new areas started " << endl;
		std::set<long long> shiftedAgents;
		for (std::map<long long, MPIAgent>::iterator it = agentsToShift.begin(); it != agentsToShift.end(); ++it) //Sending agents to destination nodes
		{
			//cout << "Agent global id: " << it->first << endl;
			float x = it->second.Position().x();
			float y = it->second.Position().y();

			//for(int ar = 0; ar < modelingAreas.size(); ar++)
			for(map<int, pair<Vector2, Vector2>> :: iterator ar = modelingAreas.begin(); ar != modelingAreas.end(); ++ar)
			{
				if(ar->second.first.x() <= x && x <= ar->second.second.x()
					&& ar->second.first.y() <= y && y <= ar->second.second.y())
				{
					int nodeID = ar->first;
					//cout << "Agent new node: " << nodeID << endl;
					MPI_Bcast(&nodeID, 1, MPI_INT, 0, MPI_COMM_WORLD); //Bcasing index of target node who should receive new agents
					//cout << "Rank: " << myRank << " Bcasting destination node: " << nodeID << endl;

					unsigned char* serializedAgent = it->second.SerializeAgent();

					int SerializedAgentSize = 0;
					memcpy(&SerializedAgentSize, serializedAgent, sizeof(int));

					MPI_Send(&SerializedAgentSize, 1, MPI_INT, nodeID, 0, MPI_COMM_WORLD);
					//cout << "Rank: " << myRank << " sending size: " << SerializedAgentSize << " to: " << nodeID << endl;
					MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, nodeID, 0, MPI_COMM_WORLD);
					//cout << "Rank: " << myRank << " agent sent." << endl;

					long long newAgentId = 0;
					//cout << "Rank: " << myRank << " agent sent." << newAgentId << endl;
					MPI_Recv(&newAgentId, 1, MPI_UNSIGNED_LONG_LONG, nodeID, 0, MPI_COMM_WORLD, MPI_STATUSES_IGNORE);

					NodesAgentsMap[AgentsIDMap[it->first]._nodeID].erase(AgentsIDMap[it->first]._agentID);
					NodesAgentsMap[nodeID][newAgentId] = it->first;

					AgentsIDMap[it->first]._nodeID = nodeID;
					AgentsIDMap[it->first]._agentID = newAgentId;

					shiftedAgents.insert(it->first);
					delete serializedAgent;
					break;
				}
			}
		}

		//cout << "Rank: " << myRank << " Bcasting finish flag: " << endl;
		destinationNode = -1;
		MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);  //End of agents shifting requests

		//Deleting agents that succesfully shifted
		std::map<long long, MPIAgent>::iterator agent_it;
		for (std::set<long long>::iterator it = shiftedAgents.begin(); it != shiftedAgents.end(); ++it)
		{
			agent_it = agentsToShift.find(*it);
			if (agent_it != agentsToShift.end())
			{
				agent_it->second.DeleteAgent();
				agentsToShift.erase(agent_it);
			}
		}

		if(agentsToShift.size() > 0)//If some of agents were outside of area but no other nodes serve for it
		{
			for (std::map<long long, MPIAgent>::iterator it = agentsToShift.begin(); it != agentsToShift.end(); ++it)
			{
				it->second.DeleteAgent();
				AgentsIDMap[it->first].isDeleted = true;
				AgentsPositions[it->first] = Vector2(INT_MIN, INT_MIN);
			}
		}

		//destinationNode = -1;
		//MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD); //End of agents shifting
		printf ("Agents shifting time: (%f seconds).\n",((float)clock() - agentsSHiftingTimeStart)/CLOCKS_PER_SEC);
	}
	else
	{
		if(myRank < modelingAreas.size() + 1)
		{
			//cout << "rank: " << myRank << " receiving agents ID to delete from simulation." << endl;
			//cout << "node: " << myRank << " moving agents crossed modeling areas started " << endl;
			do
			{
				MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD); //Check if it information about agent on this node
				//cout << "rank: " << myRank << " destination: " << destinationNode << endl;
				if (destinationNode == myRank)
				{
					size_t agentID;
					MPI_Recv(&agentID, 1, MPI_UNSIGNED_LONG_LONG, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					//cout << "rank: " << myRank << " agentID: " << agentID << endl;
					Agent* agent = simulator->getAgent(agentID);

					MPIAgent mpiTmpAgent(agent);
					unsigned char* serializedAgent = agent->Serialize();

					int SerializedAgentSize = 0;
					memcpy(&SerializedAgentSize, serializedAgent, sizeof(int));

					MPI_Send(&SerializedAgentSize, 1, MPI_INT, 0, 0, MPI_COMM_WORLD);
					//cout << "rank: " << myRank << " agent size sended:" << SerializedAgentSize << endl;

					MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, 0, 0, MPI_COMM_WORLD);
					//cout << "rank: " << myRank << " agent sent." << endl;

					simulator->deleteAgent(agentID);
					simulator->setAgentPosition(agentID, Vector2(INT_MIN, INT_MIN));
					simulator->setAgentPrefVelocity(agentID, Vector2(0, 0));
					//cout << "rank: " << myRank << " agent deleted from simulation" << endl;
					delete[] serializedAgent;
				}
			} while (destinationNode > 0);
			//cout << "rank: " << myRank << " deleting agents from simulation finished" << endl;

			//cout << "rank: " << myRank << " receiving agents to add to a simulation " << endl;
			do
			{
				MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD); //Check if it information about agent on this node
				//cout << "rank: " << myRank << " destination node: " << destinationNode << endl;
				if (destinationNode == myRank)
				{
					int SerializedAgentSize = 0; 
					MPI_Recv(&SerializedAgentSize, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

					unsigned char* buffForReceivingAgent = new unsigned char[SerializedAgentSize];
					MPI_Recv(buffForReceivingAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					//cout << "rank: " << myRank << " agent received." << endl;

					long long newAgentId = simulator->addAgent(Agent::Deseriaize(buffForReceivingAgent));
					MPI_Send(&newAgentId, 1, MPI_UNSIGNED_LONG_LONG, 0, 0, MPI_COMM_WORLD );
					//cout << "rank: " << myRank << " agent added to simulation. New ID: " << newAgentId << endl;
					delete[] buffForReceivingAgent;
				}
			} while (destinationNode > 0);
			//cout << "rank: " << myRank << " receiving agents to add to a simulation finished" << endl;
			//cout << "node: " << myRank << " moving agents crossed modeling areas finished " << endl;
		}
	}
}

void SavingModelingData()
{
	if(myRank == 0)
	{
		int savingDataStartTime = clock();
		map<long long, pair<Vector2, AgentOnNodeInfo>> iterationData;
		for (std::map<long long, AgentOnNodeInfo>::iterator it = AgentsIDMap.begin(); it != AgentsIDMap.end(); ++it)
		{
			Vector2 agentPosition(AgentsPositions[it->first].x(), AgentsPositions[it->first].y());
			AgentOnNodeInfo nodeAg(it->second._nodeID,it->second._agentID, it->second.isDeleted);
			iterationData[it->first] = pair<Vector2, AgentOnNodeInfo>(agentPosition, nodeAg);
		}
		simulationData.push_back(iterationData);
		printf ("%d rank - Saving simulating data: (%f seconds).\n", myRank,((float)clock() - savingDataStartTime)/CLOCKS_PER_SEC);
	}
}