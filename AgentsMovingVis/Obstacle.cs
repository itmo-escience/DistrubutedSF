using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Xna.Framework;

namespace AgentsMovingVis
{
    class Obstacle
    {
        public uint obstacleID { get; set; }
        public List<FloatPoint> points { get; set; }
    }
}
