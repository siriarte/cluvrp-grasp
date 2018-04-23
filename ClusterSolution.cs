using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
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

        public void verifySolution(CluVRPInstance instance)
        {
            // All cluster was visited
            bool wasVisited = false;
            for (int cluster = 0; cluster < instance.clusters().Length; cluster++)
            {
                for (int vehicle = 0; vehicle < clusterRouteForVehicule.Length; vehicle++)
                {
                    if (clusterRouteForVehicule[vehicle].Contains(cluster))
                    {
                        wasVisited = true;
                        break;
                    }
                }
                Debug.Assert(wasVisited);
                wasVisited = false;
            }

            // Number of clusters visited is correct
            int totalLength = 0;
            for (int vehicle = 0; vehicle < clusterRouteForVehicule.Length; vehicle++)
            {
                totalLength += clusterRouteForVehicule[vehicle].Count;
            }
            Debug.Assert(instance.clusters().Length == totalLength - (2 * clusterRouteForVehicule.Length) + 1);

            // Vehicle remmaining capacity is correct respect to cluster demand
            int[] clusterDemand = instance.clusters_demand();
            int totalDemand = 0;
            for (int vehicle = 0; vehicle < clusterRouteForVehicule.Length; vehicle++)
            {
                int totalDemandOnVehicle = 0;
                for (int clusterIt = 0; clusterIt < clusterRouteForVehicule[vehicle].Count; clusterIt++)
                {
                    int cluster = clusterRouteForVehicule[vehicle][clusterIt];
                    totalDemandOnVehicle += clusterDemand[cluster];
                }
                totalDemand += totalDemandOnVehicle;
                Debug.Assert(instance.capacity() - totalDemandOnVehicle >= 0);
            }
            Debug.Assert(totalDemand == clusterDemand.Sum());

        }
    }
}
