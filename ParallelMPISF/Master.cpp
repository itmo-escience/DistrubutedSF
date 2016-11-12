#include "Master.h"

Master::Master()
{
}

void Master::Run()
{
	int slaveNodesCount;
	int count;
    vector<SF::Agent> agentsRegister();
	MPI_Comm_size(MPI_COMM_WORLD, &count);

	slaveNodesCount = count - 1;

	//while()
	//{
	//	MPI_Bcast()
	//}


}

Master::~Master()
{
}
