#pragma once
#include <mpi.h>
#include <vector>
#include "../social-phys-lib-private/SF/include/SF.h"

using namespace std;
using namespace SF;

class Master
{
public:
	Master();
	void Run();
	~Master();
};



