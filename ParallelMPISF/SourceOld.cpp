#include <mpi.h>
#include <stdio.h>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <iostream>
#include <fstream>
#include "../../ParallelMPISF/social-phys-lib-private/SF/include/MPIAgent.h"
#include <set>

using namespace std;
using namespace SF;

int myRank, commSize;

void LoadData(vector<vector<SF::Vector2> > &obstacles, vector<Vector2> &agentsPositions);
void SendObstacle(vector<Vector2> obstacle);
vector<Vector2> ReceiveObstacle();
void SendAgentPosition(Vector2 agentsPosition);
Vector2 ReceiveAgentPosition();
vector<pair<Vector2, Vector2> > DivideModelingArea(const pair<Vector2, Vector2> &globalArea, float minimalWidth, float minimalHeight);
pair<Vector2, Vector2> CreateModelingArea(vector<vector<Vector2> > &obstacles, Vector2 minPoint, Vector2 maxPoint, float borderWidth);
int SendAgent(MPIAgent agent, int dest);
float GenerateRandomBetween(float LO, float HI);
void SaveObstaclesToJSON(vector<vector<Vector2> > obstacles, string path);
void SavePartitionedAreasToJSON(vector<pair<Vector2, Vector2> > modelingAreas , string path, float verticalIndent, float horizontalIndent);

struct nodeAgent {
	bool isDeleted;
	size_t _nodeID;
	size_t _agentID;
	nodeAgent() : isDeleted(false), _nodeID(-1), _agentID(-1) { }
	nodeAgent(int nodeID, int agentID) : isDeleted(false), _nodeID(nodeID), _agentID(agentID) { }
};

int main(int argc, char* argv[])
{
	MPI_Init(&argc, &argv);
	MPI_Comm_rank(MPI_COMM_WORLD, &myRank);
	MPI_Comm_size(MPI_COMM_WORLD, &commSize);
	vector<pair<Vector2, Vector2> > modelingAreas;

	if (argc != 6)
	{
		if(myRank == 0)
		{
			std::cout << "Error! Invalid number of parameters: " << endl << " min_x min_y max_x max_y agent_calc_radius" << endl;
		}
		MPI_Finalize();
		return 0;
	}

	int startTime = clock(); //programm working start moment

	std::map<long long, nodeAgent> AgentsIDMap;
	long long totalAgentsIDs = 0; 
	std::map<long long, Vector2> AgentsPositions;
	SFSimulator simulator;

//cout << "Rank: " << myRank << " Agent properties bcasting" << endl;
#pragma region AgentPropertyConfig MPI_BCasting
	unsigned char* serializedDefaultAgentConfig;
	AgentPropertyConfig* defaultAgentConfig;
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
		simulator.setAgentDefaults(*defaultAgentConfig);
	}

	delete[] serializedDefaultAgentConfig;

	//cout << "MyRank: " << myRank << " Properties: " << endl;
	//defaultAgentConfig->PrintDefaultProperties();

#pragma endregion AgentPropertyConfig MPI_BCasting

//cout << "Rank: " << myRank << " Area  partitioning" << endl;
#pragma region Modeling area partitioning
	vector<vector<Vector2> > obstacles;
	vector<Vector2> agentsPositions;

	LoadData(obstacles, agentsPositions);

	Vector2 p1(atoi(argv[1]), atoi(argv[2])); //Minimal point of modeling area
	Vector2 p2(atoi(argv[3]), atoi(argv[4])); //Maximal point of modeling area
	pair<Vector2, Vector2> globalArea = CreateModelingArea(obstacles, p1, p2, 1); //Create obstacle around modeling area (rectangle)

	//Agent radius where it interact with anothers
	float minimalWidth = atoi(argv[5]);
	float minimalHeight = atoi(argv[5]);

	modelingAreas = DivideModelingArea(globalArea, minimalWidth, minimalHeight);

	if(myRank == 0)
	{
		SaveObstaclesToJSON(obstacles, "Obstacles.txt");
		SavePartitionedAreasToJSON(modelingAreas ,"areas.txt", minimalHeight, minimalWidth);
	}
#pragma endregion Modeling area partitioning

//cout << "Rank: " << myRank << " Bcasting obstacles " << endl;
#pragma region Bcasting obstacles
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
		simulator.addObstacle(obstacles[i]);
	}
	simulator.processObstacles();
	//cout << "Rank: " << myRank << " all obstacles proceesed." << endl;
}
#pragma endregion Bcasting obstacles

#pragma region Broadcasting generated agents
		if(myRank == 0)
		{
			for (int i = 0; i < agentsPositions.size(); i++)
			{
				int destinationNode = 0;
				bool IsPOintAdded = false;
				for(int a = 0; a < modelingAreas.size(); a++) //Choosing nodes
				{
					if(agentsPositions[i].x() >= modelingAreas[a].first.x() && agentsPositions[i].x() < modelingAreas[a].second.x())
					{
						if(agentsPositions[i].y() >= modelingAreas[a].first.y() && agentsPositions[i].y() < modelingAreas[a].second.y())
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
							AgentsPositions[totalAgentsIDs] = Vector2(x, y);
							totalAgentsIDs++;
							//cout << "rank: " << myRank << " Position " << x << ":" << y << " Global ID: "<< totalAgentsIDs - 1  << " Node: "<< AgentsIDMap[totalAgentsIDs - 1]._nodeID << " ID: " << AgentsIDMap[totalAgentsIDs - 1]._agentID << endl;
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
					//if(x < )
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
	//MPI_Barrier(MPI_COMM_WORLD);

	//if(myRank == 0)
	//{
	//	cout << "My rank: " << myRank << endl;
	//	cout << "Total agents count: " << AgentsIDMap.size() << endl;

	//	typedef map<long long, Vector2>::iterator it_type;
	//	for (it_type it = AgentsPositions.begin(); it != AgentsPositions.end(); it++) {
	//		cout << "Global ID: " << it->first << " Node: " << AgentsIDMap[it->first]._nodeID << " Local ID: " << AgentsIDMap[it->first]._agentID << " x: " << it->second.x() << " y: " << it->second.y() << endl;
	//	}
	//}
	//MPI_Barrier(MPI_COMM_WORLD);

	//for(int i = 1; i < commSize; i++)
	//{
	//	if(myRank == i)
	//	{
	//		cout << "Rank: " << myRank << endl;
	//		simulator.printAgentsInfo();
	//	}
	//	MPI_Barrier(MPI_COMM_WORLD);
	//}

	//cout << "MyRank: " << myRank << endl;
#pragma endregion Printing agents info on each node
	int iterationNum = 150;
	int agentsCount = AgentsIDMap.size();
	int agNum = AgentsIDMap.size();

	std::fstream agentsPositionsFile;

	if(myRank == 0)
	{
		agentsPositionsFile.open("simData.txt", ios::out | ios::trunc);
		agentsPositionsFile << "[" << endl;
	}

	for (int iter = 0; iter < iterationNum; iter++)
	{
		int destinationNode = 0;
		long long agentId = -1;
		float xVel = 0;
		float yVel = 0;

		if (myRank == 0) //ON MAIN NODE
		{
			cout << "Iteration: " << iter << endl;
			agentsPositionsFile << "{\"iteration\": \"" << iter << "\"," << endl;

			#pragma region SENDING AGENTS VELOCITIES
			//cout << "node: " << myRank << " sending velocities started " << endl;
			
			size_t agentWithVelSize = sizeof(long long) + sizeof(float) + sizeof(float);
			for(int i = 0; i < modelingAreas.size(); i++)
			{
				//cout << "Searching agents for node " << i + 1 << endl;
				int position = 0;
				destinationNode = i + 1;
				MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
				vector<long long> agentsToSend;
				for (std::map<long long, nodeAgent>::iterator it = AgentsIDMap.begin(); it != AgentsIDMap.end(); ++it) //searching agents on node destinationNode
				{
					if(!it->second.isDeleted && it->second._nodeID == destinationNode)
					{
						agentsToSend.push_back(it->first);
					}
				}

				//cout << "Found " << agentsToSend.size() << " agents" << endl;

				buffSize = agentWithVelSize * agentsToSend.size() + sizeof(size_t);
				//cout << "Buffer size " << buffSize << endl;
				unsigned char* buffer = new unsigned char[buffSize];

				size_t agentsToSendNum = agentsToSend.size();
				MPI_Pack(&agentsToSendNum, 1, MPI_INT, buffer, buffSize, &position, MPI_COMM_WORLD);

				for(int ag = 0; ag < agentsToSend.size(); ag++) //packing agents data
				{
					agentId = AgentsIDMap[agentsToSend[ag]]._agentID;

					if (agentsToSend[ag] < agNum / 2) //Agents from zone A NOT WORKING IN COMMON CASE
					{
						if (AgentsPositions[agentsToSend[ag]].x() <= 190)
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
						if (AgentsPositions[agentsToSend[ag]].x() >= 10)
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
				//cout << "Buffer sent to " << destinationNode << endl;
				delete[] buffer;
			}

			//End of broadcasting flag
			destinationNode = -1;
			MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
			//cout << "node: " << myRank << " sending velocities finished " << endl;
			#pragma endregion sending agents velocities

			#pragma region SENDING PHANTOMS OF AGENTS THAT ON ADJACENT AREAS
			destinationNode = 0;
			//size_t agentsCount = 0;

			//cout << " minimalWidth: " << minimalWidth << " minimalHeight: " << minimalHeight << endl;
			for (std::map<long long, nodeAgent>::iterator it= AgentsIDMap.begin(); it != AgentsIDMap.end(); ++it)
			{
				if(!it->second.isDeleted)
				{
					float x = AgentsPositions[it->first].x();
					float y = AgentsPositions[it->first].y();
					//cout << "agent pos x: " << x << " y: " << y << endl;

					#pragma region left side
					if( x <= modelingAreas[it->second._nodeID - 1].first.x() + minimalWidth) // left side adjacent area
					{
						for(int i = 0; i < modelingAreas.size(); i++)
						{
							if (it->second._nodeID != i + 1)
							{
								if(modelingAreas[i].second.y() == modelingAreas[it->second._nodeID - 1].second.y()//if common side
								&& modelingAreas[i].first.y() == modelingAreas[it->second._nodeID - 1].first.y()
								&& modelingAreas[i].second.x() == modelingAreas[it->second._nodeID - 1].first.x())
								{
									//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by left side" << endl;
									//request agent
									destinationNode = it->second._nodeID;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									long long nodeAgentId = it->second._agentID;
									MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID , 0, MPI_COMM_WORLD);
									//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
									//receiving agent
									int SerializedAgentSize = 0;
									MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "agert size received" << endl;
									unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
									MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "Agent received" << endl;

									//send agent to create phantom
									destinationNode = i + 1;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									//cout << "Bcasted destination node" << endl;
									//sending agent
									MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
									MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
									delete[] serializedAgent;
								}
							}
						}
					}
					#pragma endregion left side

					#pragma region right side
					if(x >= modelingAreas[it->second._nodeID - 1].second.x() - minimalWidth) // right side adjacent area
					{
						for(int i = 0; i < modelingAreas.size(); i++)
						{
							if (it->second._nodeID != i + 1)
							{
								if(modelingAreas[it->second._nodeID - 1].second.y() == modelingAreas[i].second.y()//if common side
								&& modelingAreas[it->second._nodeID - 1].second.x() == modelingAreas[i].first.x()
								&& modelingAreas[it->second._nodeID - 1].first.y() == modelingAreas[i].first.y())
								{
									//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by right side" << endl;
									//request agent
									destinationNode = it->second._nodeID;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									long long nodeAgentId = it->second._agentID;
									MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID ,0, MPI_COMM_WORLD);
									//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
									//receiving agent
									int SerializedAgentSize = 0;
									MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "agert size received" << endl;
									unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
									MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "Agent received" << endl;

									//send agent to create phantom
									destinationNode = i + 1;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									//cout << "Bcasted destination node" << endl;
									//sending agent
									MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
									MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
									delete[] serializedAgent;
								}
							}
						}
					}
					#pragma endregion right side

					#pragma region bottom side
					if(y <= modelingAreas[it->second._nodeID - 1].first.y() + minimalHeight) // bottom side adjacent area
					{
						for(int i = 0; i < modelingAreas.size(); i++)
						{
							if (it->second._nodeID != i + 1)
							{
								if(modelingAreas[it->second._nodeID - 1].first.x() == modelingAreas[i].first.x()//if common side
								&& modelingAreas[it->second._nodeID - 1].first.y() == modelingAreas[i].second.y()
								&& modelingAreas[it->second._nodeID - 1].second.x() == modelingAreas[i].second.x())
								{
									//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by bottom side" << endl;
									//request agent
									destinationNode = it->second._nodeID;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									long long nodeAgentId = it->second._agentID;
									MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID ,0, MPI_COMM_WORLD);
									//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
									//receiving agent
									int SerializedAgentSize = 0;
									MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "agert size received" << endl;
									unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
									MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "Agent received" << endl;

									//send agent to create phantom
									destinationNode = i + 1;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									//cout << "Bcasted destination node" << endl;
									//sending agent
									MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
									MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
									delete[] serializedAgent;
								}
							}
						}
					}
					#pragma endregion bottom side

					#pragma region top side
					if(y >= modelingAreas[it->second._nodeID - 1].second.y() - minimalHeight) // top side adjacent area
					{
						for(int i = 0; i < modelingAreas.size(); i++)
						{
							if (it->second._nodeID != i + 1)
							{
								if(modelingAreas[i].first.x() == modelingAreas[it->second._nodeID - 1].first.x()//if common side
								&& modelingAreas[i].first.y() == modelingAreas[it->second._nodeID - 1].second.y()
								&& modelingAreas[i].second.x() == modelingAreas[it->second._nodeID - 1].second.x())
								{
									//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by top side" << endl;
									//request agent
									destinationNode = it->second._nodeID;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									long long nodeAgentId = it->second._agentID;
									MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID , 0, MPI_COMM_WORLD);
									//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
									//receiving agent
									int SerializedAgentSize = 0;
									MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "agert size received" << endl;
									unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
									MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "Agent received" << endl;

									//send agent to create phantom
									destinationNode = i + 1;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									//cout << "Bcasted destination node" << endl;
									//sending agent
									MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
									MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
									delete[] serializedAgent;
								}
							}
						}
					}
					#pragma endregion top side

					//diagonals
					#pragma region top right corner
					if(y >= modelingAreas[it->second._nodeID - 1].second.y() - minimalHeight 
					&& x >= modelingAreas[it->second._nodeID - 1].second.x() - minimalWidth) // top side adjacent area
					{
 						for(int i = 0; i < modelingAreas.size(); i++)
						{
							if (it->second._nodeID != i + 1)
							{
								if(modelingAreas[i].first.x() == modelingAreas[it->second._nodeID - 1].second.x()//if common side
								&& modelingAreas[i].first.y() == modelingAreas[it->second._nodeID - 1].second.y())
								{
									//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by top side" << endl;
									//request agent
 									destinationNode = it->second._nodeID;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									long long nodeAgentId = it->second._agentID;
									MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID , 0, MPI_COMM_WORLD);
									//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
									//receiving agent
									int SerializedAgentSize = 0;
									MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "agert size received" << endl;
									unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
									MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "Agent received" << endl;

									//send agent to create phantom
									destinationNode = i + 1;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									//cout << "Bcasted destination node" << endl;
									//sending agent
									MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
									MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
									delete[] serializedAgent;
								}
							}
						}
					}
					#pragma endregion top right corner

					#pragma region top left corner
					if(y >= modelingAreas[it->second._nodeID - 1].second.y() - minimalHeight 
					&& x <= modelingAreas[it->second._nodeID - 1].first.x() + minimalWidth) // top side adjacent area
					{
						for(int i = 0; i < modelingAreas.size(); i++)
						{
							if (it->second._nodeID != i + 1)
							{
								if(modelingAreas[i].second.x() == modelingAreas[it->second._nodeID - 1].first.x()//if common side
								&& modelingAreas[i].first.y() == modelingAreas[it->second._nodeID - 1].second.y())
								{
									//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by top side" << endl;
									//request agent
									destinationNode = it->second._nodeID;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									long long nodeAgentId = it->second._agentID;
									MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID , 0, MPI_COMM_WORLD);
									//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
									//receiving agent
									int SerializedAgentSize = 0;
									MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "agert size received" << endl;
									unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
									MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "Agent received" << endl;

									//send agent to create phantom
									destinationNode = i + 1;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									//cout << "Bcasted destination node" << endl;
									//sending agent
									MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
									MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
									delete[] serializedAgent;
								}
							}
						}
					}
					#pragma endregion top left corner

					#pragma region bottom left corner
					if(y <= modelingAreas[it->second._nodeID - 1].first.y() + minimalHeight 
					&& x <= modelingAreas[it->second._nodeID - 1].first.x() + minimalWidth) // top side adjacent area
					{
						for(int i = 0; i < modelingAreas.size(); i++)
						{
							if (it->second._nodeID != i + 1)
							{
								if(modelingAreas[i].second.x() == modelingAreas[it->second._nodeID - 1].first.x()//if common side
								&& modelingAreas[i].second.y() == modelingAreas[it->second._nodeID - 1].first.y())
								{
									//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by top side" << endl;
									//request agent
									destinationNode = it->second._nodeID;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									long long nodeAgentId = it->second._agentID;
									MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID , 0, MPI_COMM_WORLD);
									//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
									//receiving agent
									int SerializedAgentSize = 0;
									MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "agert size received" << endl;
									unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
									MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "Agent received" << endl;

									//send agent to create phantom
									destinationNode = i + 1;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									//cout << "Bcasted destination node" << endl;
									//sending agent
									MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
									MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
									delete[] serializedAgent;
								}
							}
						}
					}
					#pragma endregion bottom left corner

					#pragma region bottom right corner
					if(y <= modelingAreas[it->second._nodeID - 1].first.y() + minimalHeight 
					&& x >= modelingAreas[it->second._nodeID - 1].second.x() - minimalWidth) // top side adjacent area
					{
						for(int i = 0; i < modelingAreas.size(); i++)
						{
							if (it->second._nodeID != i + 1)
							{
								if(modelingAreas[i].first.x() == modelingAreas[it->second._nodeID - 1].second.x()//if common side
								&& modelingAreas[i].second.y() == modelingAreas[it->second._nodeID - 1].first.y())
								{
									//cout << "agent at position x: " << x << " " << y << " is in area x: " << modelingAreas[it->second._nodeID - 1].first.x() << " " << modelingAreas[it->second._nodeID - 1].first.y() << " " << modelingAreas[it->second._nodeID - 1].second.x() << " " << modelingAreas[it->second._nodeID - 1].second.y() << " and adjacent by top side" << endl;
									//request agent
									destinationNode = it->second._nodeID;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									long long nodeAgentId = it->second._agentID;
									MPI_Send(&nodeAgentId, 1, MPI_UNSIGNED_LONG_LONG, it->second._nodeID , 0, MPI_COMM_WORLD);
									//cout << "Agent ID to request phantom: " << nodeAgentId << endl; 
									//receiving agent
									int SerializedAgentSize = 0;
									MPI_Recv(&SerializedAgentSize, 1, MPI_INT, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "agert size received" << endl;
									unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
									MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, it->second._nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
									//cout << "Agent received" << endl;

									//send agent to create phantom
									destinationNode = i + 1;
									MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
									//cout << "Bcasted destination node" << endl;
									//sending agent
									MPI_Send(&SerializedAgentSize, 1, MPI_INT, destinationNode, 0, MPI_COMM_WORLD);
									MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, destinationNode, 0, MPI_COMM_WORLD);
									delete[] serializedAgent;
								}
							}
						}
					}
					#pragma endregion bottom right corner
				}
			}

			destinationNode = -1;
			MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);
			#pragma endregion SENDING PHANTOMS OF AGENTS THAT ON ADJACENT AREAS

			#pragma region REQUESTING NEW POSITION
			for (int areas = 0; areas < modelingAreas.size(); areas++)
			{
				int nodeID = areas + 1;
				vector<size_t> agentsIDs;
				for (std::map<long long, nodeAgent>::iterator it= AgentsIDMap.begin(); it != AgentsIDMap.end(); ++it)
				{
					if (it->second._nodeID == (nodeID)) //getting agents from current zone
					{
						agentsIDs.push_back(it->second._agentID);
					}
				}

				int requestAgentsCount = agentsIDs.size();
				MPI_Send(&requestAgentsCount, 1, MPI_INT, nodeID, 0, MPI_COMM_WORLD);

				for(int t = 0; t < requestAgentsCount; t++)
				{
					MPI_Send(&agentsIDs[t], 1, MPI_UNSIGNED, nodeID, 0, MPI_COMM_WORLD);
				}

				//Receiving serialized agent info
				for (int i = 0; i < requestAgentsCount; i++)
				{
					int SerializedAgentSize = 0;
					MPI_Recv(&SerializedAgentSize, 1, MPI_INT, nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
					MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, nodeID, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					MPIAgent agent(Agent::Deseriaize(serializedAgent));
					
					delete[] serializedAgent;
					for (std::map<long long, nodeAgent>::iterator it= AgentsIDMap.begin(); it != AgentsIDMap.end(); ++it)
					{
						if (it->second._nodeID == (nodeID) && it->second._agentID == agent.ID()) //getting agents from current zone
						{
							AgentsPositions[it->first] = Vector2(agent.Position().x(), agent.Position().y());
						}
					}
				}
			}
			#pragma endregion requesting agents new positions

			#pragma region AGENTS SHIFTING
			map<long long, MPIAgent> agentsToShift;
			//cout << "Rank: " << myRank << " Sending agents IDs for deleting from simulators"<< endl;
			for (std::map<long long, nodeAgent>::iterator it= AgentsIDMap.begin(); it != AgentsIDMap.end(); ++it)
			{
				if(!it->second.isDeleted)
				{
					float x = AgentsPositions[it->first].x();
					float y = AgentsPositions[it->first].y();
					int node = it->second._nodeID;
					if(x < modelingAreas[node - 1].first.x() || modelingAreas[node - 1].second.x() < x || //outside of its modeling area
					   y < modelingAreas[node - 1].first.y() || modelingAreas[node - 1].second.y() < y)
					{
						//cout << "agent in coords: " << x << " " << y << " not insize a area: x:" << modelingAreas[node - 1].first.x() << " y:" << modelingAreas[node - 1].first.y() << " x:" << modelingAreas[node - 1].second.x() << " y:" << modelingAreas[node - 1].second.y() << endl; 
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

			//cout << "Rank: " << myRank << " Bcasting finish flag: " << endl;
			destinationNode = -1;
			MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD);  //End of agents shifting requests

			std::set<long long> shiftedAgents;
			for (std::map<long long, MPIAgent>::iterator it = agentsToShift.begin(); it != agentsToShift.end(); ++it) //Sending agents to destination nodes
			{
				long long ID = it->second.ID();
				float x = it->second.Position().x();
				float y = it->second.Position().y();

				for(int ar = 0; ar < modelingAreas.size(); ar++)
				{
					if(modelingAreas[ar].first.x() <= x && x <= modelingAreas[ar].second.x()
						&& modelingAreas[ar].first.y() <= y && y <= modelingAreas[ar].second.y())
					{
						int nodeID = ar + 1;
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
						AgentsIDMap[it->first]._nodeID = nodeID;
						AgentsIDMap[it->first]._agentID = newAgentId;
						shiftedAgents.insert(it->first);
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
						agentsToShift.erase(agent_it);
					}
			}

			if(agentsToShift.size() > 0)//If some of agents were outside of area but no other nodes serve for it
			{
				for (std::map<long long, MPIAgent>::iterator it = agentsToShift.begin(); it != agentsToShift.end(); ++it)
				{
					AgentsIDMap[it->first].isDeleted = true;
					AgentsPositions[it->first] = Vector2(INT_MIN, INT_MIN);
				}
			}

			//destinationNode = -1;
			//MPI_Bcast(&destinationNode, 1, MPI_INT, 0, MPI_COMM_WORLD); //End of agents shifting
			#pragma endregion Agents shifting

			//MPI_Barrier(MPI_COMM_WORLD);
			agentsPositionsFile << "\"agents\": [" << endl;
			bool notFirseAgentInSection = false;
			for (std::map<long long, nodeAgent>::iterator it = AgentsIDMap.begin(); it != AgentsIDMap.end(); ++it)
			{
				if (!it->second.isDeleted)
				{
					if(notFirseAgentInSection)
					{
						agentsPositionsFile << "," << endl;
					}

					agentsPositionsFile << "{" << endl;
					agentsPositionsFile << "\t\"agentID\" : \"" << it->first << "\"," <<endl;
					agentsPositionsFile << "\t\"agentCurrentNode\" : \"" << AgentsIDMap[it->first]._nodeID << "\"," <<endl;
					agentsPositionsFile << "\t\"X\" : \"" << AgentsPositions[it->first].x() << "\"," << endl;
					agentsPositionsFile << "\t\"Y\" : \"" << AgentsPositions[it->first].y() << "\"" << endl;
					agentsPositionsFile << "}";
					
					notFirseAgentInSection = true;
				}
			}
			agentsPositionsFile << "\n]" << endl;
		}
		else //ON SLAVE NODE
		{
			if(myRank < modelingAreas.size() + 1)
			{
				#pragma region RECEIVING VELOCITIES
			//cout << "node: " << myRank << " receiving velocities started " << endl;
			do
			{
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
						agentsCount = 0;
						MPI_Unpack(buffer, buffSize, &position, &agentsCount, 1, MPI_INT, MPI_COMM_WORLD);
						//cout << "In a buffer agents count " << agentsCount << endl;
						for(int i = 0; i < agentsCount; i++)
						{
							MPI_Unpack(buffer, buffSize, &position, &agentId, 1, MPI_LONG_LONG_INT, MPI_COMM_WORLD);
							MPI_Unpack(buffer, buffSize, &position, &xVel, 1, MPI_FLOAT, MPI_COMM_WORLD);
							MPI_Unpack(buffer, buffSize, &position, &yVel, 1, MPI_FLOAT, MPI_COMM_WORLD);
							//cout << "Agent unpacked ID: " << agentId << " xvel: " << xVel << " yVel: " << yVel << endl;
							simulator.setAgentPrefVelocity(agentId, Vector2(xVel, yVel));
						}
						//cout << "All received agents added to model" << endl;
						delete[] buffer;
					}
			} while (destinationNode > 0);
			//cout << "node: " << myRank << " receiving velocities finished " << endl;
			#pragma endregion RECEIVING VELOCITIES

				#pragma region RECEIVING AGENTS FROM ADJACENT AREA
			int destination = 0;
			//cout << "node: " << myRank << " receiving agents from adjacent areas started " << endl;
			bool ImReceivingPhantom = false;
			do
			{
				//cout << "waiting bcast" << endl;
				MPI_Bcast(&destination, 1, MPI_INT, 0, MPI_COMM_WORLD);
				//cout << "Received destination: " << destination << endl;
				if(destination == myRank)
				{
					if(!ImReceivingPhantom)
					{
						//cout << "Sending phantom" << endl;
						long long agentIDtoSend = 0;
						MPI_Recv(&agentIDtoSend, 1, MPI_UNSIGNED_LONG_LONG, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
						//cout << "Rank: " << myRank << " received ID:" << agentIDtoSend << endl;
						//Send agent to create phantom on adjacent node
						unsigned char* agentBuffer = simulator.getAgent(agentIDtoSend)->Serialize();
						int SerializedAgentSize = 0;
						memcpy(&SerializedAgentSize, agentBuffer, sizeof(int));
						//cout << "Rank: " << myRank << " serialized agent size:" << SerializedAgentSize << endl;
						MPI_Send(&SerializedAgentSize, 1, MPI_INT, 0, 0, MPI_COMM_WORLD);
						MPI_Send(agentBuffer, SerializedAgentSize, MPI_UNSIGNED_CHAR, 0, 0, MPI_COMM_WORLD);
						//cout << "Rank: " << myRank << " serialized agent to the main node sent" << endl;
					}
					else
					{
						//receiving phantom
						//cout << "Receiving a phantom: " << endl;
						int SerializedAgentSize = 0;
						MPI_Recv(&SerializedAgentSize, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
						//cout << "Rank: " << myRank << " receiving agent with size: " << SerializedAgentSize << endl;
						unsigned char* serializedAgent = new unsigned char[SerializedAgentSize];
						MPI_Recv(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
						//cout << "Rank: " << myRank << " agent received " << SerializedAgentSize << endl;
						simulator.addTempAgent(Agent::Deseriaize(serializedAgent));
						//cout << "Rank: " << myRank << " phantom added to a model" << endl;
					}
				}
				else
				{
					//cout << "Not my destination" << endl;
				}
				ImReceivingPhantom = !ImReceivingPhantom;

			}
			while (destination > 0);
			//cout << "node: " << myRank << " receiving agents from adjacent areas finished " << endl;
			#pragma endregion RECEIVING AGENTS FROM ADJACENT AREA

				#pragma region SIMULATING A STEP
			//cout << "node: " << myRank << " iteration: "  << iter << endl;
			//cout << "node: " << myRank << " simulation step started finished " << endl;
			simulator.doStep();
			//cout << "node: " << myRank << " simulation step finished finished " << endl;
			#pragma endregion SIMULATING A STEP

				#pragma region SENDING NEW POSITIONS OF AGENTS
			//Requesting vector with requesting agents IDs
			int requestedAgentsCount;
			vector<size_t> requestedAgentsIDs;
			//cout << "node: " << myRank << " waiting to request agents count" << endl;
			MPI_Recv(&requestedAgentsCount, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
			//cout << "node: " << myRank << " requested agents count: " << requestedAgentsCount << endl;
			for(int t  = 0; t < requestedAgentsCount; t++)
			{
				int requestedID = 0;
				MPI_Recv(&requestedID, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
				requestedAgentsIDs.push_back(requestedID);
			}
			//cout << "node: " << myRank << " requesting " << requestedAgentsCount << " agents ids finished "<< endl;

			std::map<size_t, Vector2> agentsNewPositions;
			for (int i = 0; i < requestedAgentsIDs.size(); i++)
			{
				Agent* tmpAgent = simulator.getAgent(requestedAgentsIDs[i]);
				MPIAgent mpiTmpAgent(tmpAgent);
				unsigned char* serializedAgent = tmpAgent->Serialize();

				int SerializedAgentSize = 0;
				memcpy(&SerializedAgentSize, serializedAgent, sizeof(int));

				MPI_Send(&SerializedAgentSize, 1, MPI_INT, 0, 0, MPI_COMM_WORLD);
				MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, 0, 0, MPI_COMM_WORLD);
				delete[] serializedAgent;
			}
			
			//cout << "node: " << myRank << " sending " << requestedAgentsCount << " agents finished "<< endl;
			#pragma endregion SENDING NEW POSITIONS OF AGENTS

				#pragma region MOVING AGENTS CROSSED MODELIG AREAS
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
					Agent* agent = simulator.getAgent(agentID);

					MPIAgent mpiTmpAgent(agent);
					unsigned char* serializedAgent = agent->Serialize();

					int SerializedAgentSize = 0;
					memcpy(&SerializedAgentSize, serializedAgent, sizeof(int));

					MPI_Send(&SerializedAgentSize, 1, MPI_INT, 0, 0, MPI_COMM_WORLD);
					//cout << "rank: " << myRank << " agent size sended:" << SerializedAgentSize << endl;

					MPI_Send(serializedAgent, SerializedAgentSize, MPI_UNSIGNED_CHAR, 0, 0, MPI_COMM_WORLD);
					//cout << "rank: " << myRank << " agent sent." << endl;

					simulator.deleteAgent(agentID);
					simulator.setAgentPosition(agentID, Vector2(INT_MIN, INT_MIN));
					simulator.setAgentPrefVelocity(agentID, Vector2(0, 0));
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

					long long newAgentId = simulator.addAgent(Agent::Deseriaize(buffForReceivingAgent));
					MPI_Send(&newAgentId, 1, MPI_UNSIGNED_LONG_LONG, 0, 0, MPI_COMM_WORLD );
					//cout << "rank: " << myRank << " agent added to simulation. New ID: " << newAgentId << endl;
					delete[] buffForReceivingAgent;
				}
			} while (destinationNode > 0);
			//cout << "rank: " << myRank << " receiving agents to add to a simulation finished" << endl;
			//cout << "node: " << myRank << " moving agents crossed modeling areas finished " << endl;
			#pragma endregion MOVING AGENTS CROSSED MODELIG AREAS
			}
			else
			{
				//cout << "No modeling area for node: " << myRank << endl;
			}
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

	int workTime = clock() - startTime; 
	printf ("program working time: %d clicks (%f seconds).\n",workTime,((float)workTime)/CLOCKS_PER_SEC);

	return 0;
}

void LoadData(vector<vector<Vector2> > &obstacles, vector<Vector2> &agentsPositions)
{
	int agentsInAZone = 100;
	int agentsInBZone = 100;

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
		srand(time(NULL));

		//random agents in zone A
		for (int i = 0; i < agentsInAZone; i++)
		{
			Vector2 agentPosition(GenerateRandomBetween(zomeAMinPoint.x(), zomeAMaxPoint.x()), GenerateRandomBetween(zomeAMinPoint.y(), zomeAMaxPoint.y()));
				//zomeAMinPoint.x() + rand() % (int)zomeAMaxPoint.x(), zomeAMinPoint.y() + rand() % (int)zomeAMaxPoint.y());
			agentsPositions.push_back(agentPosition);
		}

		//random agents in zone B
		for (int i = 0; i < agentsInBZone; i++)
		{
			Vector2 agentPosition(GenerateRandomBetween(zomeBMinPoint.x(), zomeBMaxPoint.x()), GenerateRandomBetween(zomeBMinPoint.y(), zomeBMaxPoint.y()));
				//zomeBMinPoint.x() + rand() % (int)zomeBMaxPoint.x(), zomeBMinPoint.y() + rand() % (int)zomeBMaxPoint.y());
			agentsPositions.push_back(agentPosition);
		}

		//Vector2 ag1(99, 49);
		//Vector2 ag2(99, 51);

		//Vector2 ag3(101, 51);	
		//Vector2 ag4(101, 49);

		//Vector2 ag1(105, 10);
		//Vector2 ag2(105, 10);

		//Vector2 ag3(105, 20);	
		//Vector2 ag4(106, 20);

		//Vector2 ag5(105, 20);	
		//Vector2 ag6(108, 20);

		//Vector2 ag3(50, 52);
		//Vector2 ag4(50, 48);

		//Vector2 ag3(50, 52);
		//Vector2 ag4(50, 48);

		//agentsPositions.push_back(ag1);
		//agentsPositions.push_back(ag2);

		//agentsPositions.push_back(ag1);
		//agentsPositions.push_back(ag2);

		//agentsPositions.push_back(ag3);
		//agentsPositions.push_back(ag4);

		//agentsPositions.push_back(ag5);
		//agentsPositions.push_back(ag6);

		//agentsPositions.push_back(ag3);
		//agentsPositions.push_back(ag4);
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
vector<pair<Vector2, Vector2> > DivideModelingArea(const pair<Vector2, Vector2> &globalArea, float minimalWidth, float minimalHeight)
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

void SavePartitionedAreasToJSON(vector<pair<Vector2, Vector2> > modelingAreas , string path, float verticalIndent, float horizontalIndent)
{
	std::fstream modelingSubareasFile;
	modelingSubareasFile.open(path.c_str(), ios::out | ios::trunc);
	modelingSubareasFile << "[" << endl;
	for(int i = 0; i < modelingAreas.size(); i++)
	{
		modelingSubareasFile << "{" << endl;
		
		modelingSubareasFile << "\"verticalIndent\":" << verticalIndent << "," << endl;
		modelingSubareasFile << "\"horizontalIndent\":" << horizontalIndent << "," << endl;
		modelingSubareasFile << "\"x1\":" << modelingAreas[i].first.x() << "," << endl;
		modelingSubareasFile << "\"y1\":" << modelingAreas[i].first.y() << "," << endl;
		modelingSubareasFile << "\"x2\":" << modelingAreas[i].second.x() << "," << endl;
		modelingSubareasFile << "\"y2\":" << modelingAreas[i].second.y() << endl;

		if(i < modelingAreas.size() - 1)
		{
			modelingSubareasFile << "}," << endl;
		}
		else
		{
			modelingSubareasFile << "}" << endl;
		}
	}

	modelingSubareasFile << "]" << endl;
	modelingSubareasFile.close();
}

void SaveObstaclesToJSON(vector<vector<Vector2> > obstacles, string path)
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
