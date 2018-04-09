using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace cluvrp_grasp
{
    class CluVRPSolution
    {
        public double totalDistance;
        public List<int>[] clustersRoute;
        public List<int>[] nodesRoute;
        public int[] vehiculeCapacity;

        public CluVRPSolution(List<int>[] clustersRoute, List<int>[] nodesRoute, double totalDistance, int[] vehiculeCapacity)
        {
            this.clustersRoute = clustersRoute;
            this.nodesRoute = nodesRoute;
            this.totalDistance = totalDistance;
            this.vehiculeCapacity = vehiculeCapacity;
        }

        public CluVRPSolution()
        {
        }


    }
}
