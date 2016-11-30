using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AgentsMovingVis
{
    public class Agent
    {
        public long agentID { get; set; }
        public int agentCurrentNode { get; set; }
        public float X { get; set; }
        public float Y { get; set; }
    }
}