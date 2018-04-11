using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace cluvrp_grasp
{

    class ClusterSolution
    {
        public List<int>[] clusterRouteForVehicule { set; get; }
        public double totalRouteDistance { set; get; }

        public ClusterSolution(List<int>[] clusterRouteForVehicule, double totalRouteDistance)
        {
            this.clusterRouteForVehicule = clusterRouteForVehicule;
            this.totalRouteDistance = totalRouteDistance;
        }
    }
}
