#pragma once
class AgentOnNodeInfo
{
public:
	AgentOnNodeInfo(void): isDeleted(false), _nodeID(-1), _agentID(-1) { };
	bool isDeleted;
	long long _nodeID;
	long long _agentID;
	AgentOnNodeInfo(int nodeID, long long agentID) : isDeleted(false), _nodeID(nodeID), _agentID(agentID) { }
	AgentOnNodeInfo(int nodeID, long long agentID, bool isDeleted) : isDeleted(isDeleted), _nodeID(nodeID), _agentID(agentID) { }
	~AgentOnNodeInfo(void);
};

