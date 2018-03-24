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

            for (int iteration = 0; iteration < iterationsForBestSolution; iteration++)
            {
                if (!verifyClustersDemand(instance))
                {
                    Logger.GetInstance().logLine("Capacity demanded can't be served by the vehicules");
                }

                double[][] a = calculateInterClusterDistance(instance);

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

        static private double[][] calculateInterClusterDistance(CluVRPInstance instance)
        {
            double[][] nodesDistanceMatrix = new double[instance.dimension()][];
            NodePoint[] nodesPosition = instance.nodes();

            for (int i = 0; i < instance.dimension(); i++)
            {
                nodesDistanceMatrix[i] = new double[instance.dimension()];
                for (int j = 0; j < instance.dimension(); j++)
                {
                    nodesDistanceMatrix[i][j] = distance(nodesPosition[i].getX(), nodesPosition[i].getY(), nodesPosition[j].getX(), nodesPosition[j].getY());
                }
            }

            double[][] clusterDistanceMatrix = new double[instance.dimension()][];
            int[][] clusters = instance.clusters();

            for(int i = 0; i < clusters[i].Length; i++)
            {
                for(int j = 1; j < clusters[j].Length; j++)
                {
                    
                }
            }




            return null;            
        }

        static public double distance(double x1, double y1, double x2, double y2)
        {
            return Math.Sqrt(Math.Pow(x1-x2, 2) + Math.Pow(y1-y2, 2));
        }
    }
}
