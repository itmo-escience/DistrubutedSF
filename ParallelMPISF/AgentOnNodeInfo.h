#pragma once
class AgentOnNodeInfo
{
public:
	AgentOnNodeInfo(void): isDeleted(false), _nodeID(-1), _agentID(-1) { };
	bool isDeleted;
	int _nodeID;
	long long _agentID;
	AgentOnNodeInfo(int nodeID, int agentID) : isDeleted(false), _nodeID(nodeID), _agentID(agentID) { }
	AgentOnNodeInfo(int nodeID, int agentID, bool isDeleted) : isDeleted(isDeleted), _nodeID(nodeID), _agentID(agentID) { }
	~AgentOnNodeInfo(void);
};

