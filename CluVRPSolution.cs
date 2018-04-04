using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CluVRP_GRASP
{
    class CluVRPSolution
    {
        public double totalDistance;
        public List<int>[] clustersRoute;
        public List<int>[] nodesRoute;

        public CluVRPSolution(List<int>[] clustersRoute, List<int>[] nodesRoute, double totalDistance)
        {
            this.clustersRoute = clustersRoute;
            this.nodesRoute = nodesRoute;
            this.totalDistance = totalDistance;           
        }

        public CluVRPSolution()
        {
        }


    }
}
