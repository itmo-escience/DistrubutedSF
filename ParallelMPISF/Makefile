dsf: Agent.o KdTree.o AgentPropertyConfig.o MPIAgent.o Obstacle.o SFSimulator.o SimpleMatrix.o AgentOnNodeInfo.o main.o
	mpicxx -g -rdynamic Agent.o KdTree.o AgentPropertyConfig.o MPIAgent.o Obstacle.o SFSimulator.o SimpleMatrix.o AgentOnNodeInfo.o Source.o -o dsf2

all: main.o Agent.o KdTree.o AgentPropertyConfig.o MPIAgent.o Obstacle.o SFSimulator.o SimpleMatrix.o AgentOnNodeInfo.o Source.o
	icpc -std=c++0x -g -rdynamic -O2 Agent.o KdTree.o AgentPropertyConfig.o MPIAgent.o Obstacle.o SFSimulator.o SimpleMatrix.o AgentOnNodeInfo.o Source.o -o out

Agent.o: SF/src/Agent.cpp
	icpc -std=c++0x -g -rdynamic -c -O2 SF/src/Agent.cpp

KdTree.o: SF/src/KdTree.cpp
	icpc -std=c++0x -g -rdynamic -c -O2 SF/src/KdTree.cpp

AgentPropertyConfig.o: SF/src/AgentPropertyConfig.cpp
	icpc -std=c++0x -g -rdynamic -c -O2 SF/src/AgentPropertyConfig.cpp
	
	
AgentOnNodeInfo.o: AgentOnNodeInfo.cpp
	icpc -std=c++0x -g -rdynamic -c -O2 AgentOnNodeInfo.cpp

MPIAgent.o: SF/src/MPIAgent.cpp
	icpc -std=c++0x -g -rdynamic -c -O2 SF/src/MPIAgent.cpp

Obstacle.o: SF/src/Obstacle.cpp
	icpc -std=c++0x -g -rdynamic -c -O2 SF/src/Obstacle.cpp

SFSimulator.o: SF/src/SFSimulator.cpp
	icpc -std=c++0x -g -rdynamic -c -O2 SF/src/SFSimulator.cpp

SimpleMatrix.o: SF/src/SimpleMatrix.cpp
	icpc -std=c++0x -g -rdynamic -c -O2 SF/src/SimpleMatrix.cpp
	
Source.o: Source.cpp
	icpc -std=c++0x -g -rdynamic -c -O2 Source.cpp

main.o: Source.cpp
	mpicxx -g -rdynamic -c -O2 Source.cpp

sf:
	icpc -std=c++0x -g -rdynamic -c -O2 SF/src/Agent.cpp  SF/src/AgentPropertyConfig.cpp SF/src/KdTree.cpp SF/src/MPIAgent.cpp SF/src/Obstacle.cpp SF/src/SFSimulator.cpp SF/src/SimpleMatrix.cpp

clean:
	rm *.o

clean_all:
	rm *.o dsf
