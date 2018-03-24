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

                Tuple<int, int, double>[][][] a = calculateInterClusterMatrixDistance(instance);
                a = null;

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

        static private double[][] calculateNodesMatrixDistance(CluVRPInstance instance)
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
            return nodesDistanceMatrix;
        }

        static private Tuple<int, int, double>[][][] calculateInterClusterMatrixDistance(CluVRPInstance instance)
        {
            Tuple<int, int, double>[][][] clusterDistanceMatrix = new Tuple<int, int, double>[instance.clusters().Length][][];
            int[][] clusters = instance.clusters();
            double[][] nodesDistanceMatrix = calculateNodesMatrixDistance(instance);

            for (int i = 0; i<clusters.Length; i++)
            {
                clusterDistanceMatrix[i] = new Tuple<int, int, double>[clusters.Length][];
                for (int j = 0; j<clusters.Length; j++)
                {
                    clusterDistanceMatrix[i][j] = new Tuple<int, int, double>[clusters[i].Length * clusters[j].Length];
                    int idx = 0;
                    for (int k = 0; k < clusters[i].Length; k++)
                    {
                        for (int p = 0; p < clusters[j].Length; p++)
                        {
                            Tuple<int, int, double> t = new Tuple<int, int, double>(clusters[i][k], clusters[j][p], nodesDistanceMatrix[clusters[i][k]][clusters[j][p]]);
                            clusterDistanceMatrix[i][j][idx] = t;
                            idx++;
                        }
                    }
                }
            }
            return clusterDistanceMatrix;
          }
        

        static public double distance(double x1, double y1, double x2, double y2)
        {
            return Math.Sqrt(Math.Pow(x1-x2, 2) + Math.Pow(y1-y2, 2));
        }
    }
}
