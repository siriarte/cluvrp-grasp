using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CluVRP_GRASP
{
    static class Grasp
    {

        static public void ConstructGreedySolution(CluVRPInstance instance, int iterationsForBestSolution)
        {
            float maxSolution = float.MaxValue;
            
            for(int iteration = 0; iteration < iterationsForBestSolution; iteration++)
            {
                if(!verifyClustersDemand(instance))
                {
                    Logger.GetInstance().logLine("Capacity demmanded can't be served by the vehicules");
                }
            }


            return;
        }

        // Verify if the clusters can be served by the sum of the vechicules capacity
        static private bool verifyClustersDemand(CluVRPInstance instance)
        {
            int availableCapacity = instance.vehicules() * instance.capacity();
            int[] clusterDemand = instance.clusters_demand();
            return availableCapacity >= clusterDemand.Sum();
        }
    }
}
