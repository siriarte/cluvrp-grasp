using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NodeTupleDistance = System.Tuple<int, int, double>;

namespace CluVRP_GRASP
{
   
    static class Grasp
    {
        static public double ConstructGreedySolution(CluVRPInstance instance, int iterationsForBestSolution)
        {
            float maxSolution = float.MaxValue;
            double[][] nodeDistance = calculateNodesMatrixDistance(instance);
            int[][] clusters = instance.clusters();
            double totalDistance = 0;
            NodeTupleDistance[][][] interClusterDistances = calculateInterClusterMatrixDistance(instance);
            List<int>[] vehiculeAssignation = assignVehicules(instance, interClusterDistances);

            for (int iteration = 0; iteration < iterationsForBestSolution; iteration++)
            {
                if (!verifyClustersDemand(instance))
                {
                    Logger.GetInstance().logLine("Capacity demanded can't be served by the vehicules");
                }

                totalDistance = 0;
                for (int vehiculeIndex = 0; vehiculeIndex < vehiculeAssignation.Length; vehiculeIndex++)
                {
                    int actualNode = 0;
                    int actualCluster = 0;

                    List<int> clusterToVisit = vehiculeAssignation[vehiculeIndex];
                    clusterToVisit.Remove(actualCluster);
                    for(int clusterIt = 0; clusterIt < clusterToVisit.Count; clusterIt++)
                    {
                        Tuple<int, int, double> nextClusterAndNode = selectNextClusterAndNode(actualNode, actualCluster, clusterToVisit, interClusterDistances);
                        actualNode = nextClusterAndNode.Item1;
                        actualCluster = nextClusterAndNode.Item2;
                        totalDistance += nextClusterAndNode.Item3;

                        Tuple<List<int>, double> intraClusterTravel = calculateIntraClusterTravel(actualNode, actualCluster, clusters[actualCluster], interClusterDistances);
                        totalDistance += intraClusterTravel.Item2;
                        actualNode = intraClusterTravel.Item1[intraClusterTravel.Item1.Count - 1];
                    }
                    totalDistance += nodeDistance[actualNode][0];
                }

            }
            return totalDistance;
        }

        static private Tuple<int, int, double> selectNextClusterAndNode(int actualNode, int actualCluster, List<int>clusterToVisit, NodeTupleDistance[][][] interClusterDistances)
        {
            NodeTupleDistance[][] clustersDistance = interClusterDistances[actualCluster];
            Tuple<int, int, double> bestTravel = new Tuple<int, int, double>(actualNode, actualCluster, double.MaxValue);
            for (int nextCluster = 0; nextCluster < clustersDistance.Length; nextCluster++)
            {
                if (nextCluster != actualCluster && clusterToVisit.Contains(nextCluster))
                {
                    for (int nodeDistanceIt = 0; nodeDistanceIt < clustersDistance[nextCluster].Length; nodeDistanceIt++)
                    {
                        if (clustersDistance[nextCluster][nodeDistanceIt].Item1 == actualNode)
                        {
                            if (bestTravel.Item3 > clustersDistance[nextCluster][nodeDistanceIt].Item3)
                            {
                                bestTravel = new Tuple<int, int, double>(clustersDistance[nextCluster][nodeDistanceIt].Item2, nextCluster, clustersDistance[nextCluster][nodeDistanceIt].Item3);
                            }
                        }
                        if (clustersDistance[nextCluster][nodeDistanceIt].Item2 == actualNode)
                        {
                            if (bestTravel.Item3 > clustersDistance[nextCluster][nodeDistanceIt].Item3)
                            {
                                bestTravel = new Tuple<int, int, double>(clustersDistance[nextCluster][nodeDistanceIt].Item1, nextCluster, clustersDistance[nextCluster][nodeDistanceIt].Item3);
                            }
                        }
                    }
                }
            }
            return bestTravel;
        }


        static private Tuple<List<int>, double> calculateIntraClusterTravel(int actualNode, int actualCluster, int[] nodesOnCluster, NodeTupleDistance[][][] interClusterDistances)
        {
            NodeTupleDistance[] intraClusterDistance = interClusterDistances[actualCluster][actualCluster];
            double totalDistance = 0;
            List<int> intraClusterTravel = new List<int>();
            intraClusterTravel.Add(actualNode);
        

            for (int nodesVisited = 1; nodesVisited < nodesOnCluster.Length; nodesVisited++)
            {
                double bestDistance = double.MaxValue;

                for (int it = 0; it < intraClusterDistance.Length; it++)
                {
                    if (intraClusterDistance[it].Item2 != actualNode && intraClusterDistance[it].Item1 == actualNode && 
                        intraClusterDistance[it].Item3 < bestDistance && !intraClusterTravel.Contains(intraClusterDistance[it].Item2))
                    {
                        actualNode = intraClusterDistance[it].Item2;
                        bestDistance = intraClusterDistance[it].Item3;
  
                    }
                    if (intraClusterDistance[it].Item1 != actualNode && intraClusterDistance[it].Item2 == actualNode 
                        && intraClusterDistance[it].Item3 < bestDistance && !intraClusterTravel.Contains(intraClusterDistance[it].Item1))
                    {
                        actualNode = intraClusterDistance[it].Item1;
                        bestDistance = intraClusterDistance[it].Item3;
                     }
                }
                intraClusterTravel.Add(actualNode);
                totalDistance += bestDistance;
            }
            return new Tuple<List<int>, double>(intraClusterTravel, totalDistance);
        }

        // Verify if the clusters can be served by the sum of the vechicules capacity
        static private bool verifyClustersDemand(CluVRPInstance instance)
        {
            int availableCapacity = instance.vehicules() * instance.capacity();
            int[] clusterDemand = instance.clusters_demand();
            return availableCapacity >= clusterDemand.Sum();
        }


        // Create a nodes distance matrix
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

        // Create a inter-clusters distance array 
        // [i][j] is the array of tubles <node1, node2, distance> of distances between all nodes
        // on the i-cluster and j-cluster
        static private NodeTupleDistance[][][] calculateInterClusterMatrixDistance(CluVRPInstance instance)
        {
            double[][] nodesDistanceMatrix = calculateNodesMatrixDistance(instance);
            int clustersNumber = instance.clusters_demand().Length;
            NodeTupleDistance[][][] clusterDistanceMatrix = new Tuple<int, int, double>[clustersNumber][][];
            int[][] clusters = instance.clusters();
           
            for (int i = 0; i < clustersNumber; i++)
            {
                clusterDistanceMatrix[i] = new Tuple<int, int, double>[clustersNumber][];
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
        
        // Distance function
        static public double distance(double x1, double y1, double x2, double y2)
        {
            return Math.Sqrt(Math.Pow(x1-x2, 2) + Math.Pow(y1-y2, 2));
        }

        // Return an assignation of nodes to vehicules
        static private List<int>[] assignVehicules(CluVRPInstance instance, NodeTupleDistance[][][] interClusterMatrixDistance)
        {
            int[] clusterDemand = instance.clusters_demand();
            int vehiculesNumber = instance.vehicules();
            int capacity = instance.capacity();
            return assignVehiculesBestFitAlgorithm(clusterDemand, vehiculesNumber, capacity);
        }

        static private List<int>[] assignVehiculesBestFitAlgorithm(int[] clusterDemand, int vehiculesNumber, int capacity)
        {
            List<int>[] clusterRouteForVehicule = new List<int>[vehiculesNumber];
            int[] vehiculeCapacity = new int[vehiculesNumber];
            for (int i = 0; i < vehiculesNumber; i++)
            {
                vehiculeCapacity[i] = capacity;
                clusterRouteForVehicule[i] = new List<int>();
            }

            int[] indexSortedClustedDemand = arraySortedByIndex(clusterDemand);
            for (int i = 0; i < clusterDemand.Length; i++)
            {
                int minCapacityIndex = indexSortedClustedDemand[i];
                for(int j = 0; j < vehiculeCapacity.Length; j++)
                {
                    if(vehiculeCapacity[j] - clusterDemand[minCapacityIndex] >= 0)
                    {
                        clusterRouteForVehicule[j].Add(minCapacityIndex);
                        vehiculeCapacity[j] = vehiculeCapacity[j] - clusterDemand[minCapacityIndex];
                        break;
                    } 
                }
            }
            return clusterRouteForVehicule;
        }


        static private int[] arraySortedByIndex(int[] arr)
        {
            int[] ret = new int[arr.Length];
            for(int i = 0; i < arr.Length; i++)
            {
                ret[i] = i;
            }

            for(int i = 0; i < arr.Length; i++)
            {
                for(int j = i + 1; j < arr.Length; j++)
                {
                    if (arr[ret[j]] <= arr[ret[i]])
                    {
                        int temp = ret[i];
                        ret[i] = ret[j];
                        ret[j] = temp;
                    }
                }
            }
            return ret;
        }
    }
}
