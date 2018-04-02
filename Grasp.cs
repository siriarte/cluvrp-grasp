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
        static public double ConstructGreedySolution(CluVRPInstance instance, int iterationsForBestSolution = 10)
        {
 
            double[][] nodesMatrixDistance = calculateNodesMatrixDistance(instance);
            bool[][] clustersMatrix = Grasp.clustersMatrix(instance.clusters(), instance.dimension());
            double bestDistance = double.MaxValue;
            double totalDistance = 0;

            for (int iteration = 0; iteration < iterationsForBestSolution; iteration++)
            {
                if (!verifyClustersDemand(instance))
                {
                    Logger.GetInstance().logLine("Capacity demanded can't be served by the vehicules");
                }

                totalDistance = 0;
                List<int>[] vehiculeAssignation = assignVehicules(instance);

                for (int vehiculeIndex = 0; vehiculeIndex < vehiculeAssignation.Length; vehiculeIndex++)
                {
                    int actualNode = 0;
                    int actualCluster = 0;

                    List<int> clustersToVisit = vehiculeAssignation[vehiculeIndex];
                    for(int clusterIt = 0; clusterIt < clustersToVisit.Count; clusterIt++)
                    {
                        clustersToVisit.Remove(actualCluster);
                        Tuple<int, int, double> nextClusterAndNode = selectNextClusterAndNode(actualNode, actualCluster, clustersToVisit, nodesMatrixDistance, clustersMatrix);
                        actualNode = nextClusterAndNode.Item1;
                        actualCluster = nextClusterAndNode.Item2;
                        totalDistance += nextClusterAndNode.Item3;

                        Tuple<List<int>, double> intraClusterTravel = calculateIntraClusterTravel(actualNode, actualCluster, instance.clusters()[actualCluster], nodesMatrixDistance);
                        totalDistance += intraClusterTravel.Item2;
                        actualNode = intraClusterTravel.Item1[intraClusterTravel.Item1.Count - 1];
                        //Logger.GetInstance().logLine(String.Join(",", intraClusterTravel.Item1));
                    }
                    
                    totalDistance += nodesMatrixDistance[actualNode][0];
                }

                // Update best solution
                if (totalDistance < bestDistance)
                {
                    bestDistance = totalDistance;
                }
            }
            return bestDistance;
        }

        // Return an assignation of nodes to vehicules
        static private List<int>[] assignVehicules(CluVRPInstance instance)
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
                // add base (cluster 0)
                clusterRouteForVehicule[i].Add(0);
            }

            int[] indexSortedClustedDemand = arraySortedByIndex(clusterDemand);
            for (int i = 1; i < clusterDemand.Length; i++)
            {
                int minCapacityIndex = indexSortedClustedDemand[i];
                for (int j = 0; j < vehiculeCapacity.Length; j++)
                {
                    if (vehiculeCapacity[j] - clusterDemand[minCapacityIndex] >= 0)
                    {
                        clusterRouteForVehicule[j].Add(minCapacityIndex);
                        vehiculeCapacity[j] = vehiculeCapacity[j] - clusterDemand[minCapacityIndex];
                        break;
                    }
                }
            }
            return clusterRouteForVehicule;
        }
        static private List<int>[] assignVehiculesBestFitRandomizedAlgorithm(int[] clusterDemand, int vehiculesNumber, int capacity, int rclsize = 3)
        {
            List<int>[] clusterRouteForVehicule = new List<int>[vehiculesNumber];
            bool[] visitedCluster = new bool[clusterDemand.Length];
            int[] vehiculeCapacity = new int[vehiculesNumber];
            int totalDemand = clusterDemand.Sum();
            int totalClusterVisited = 0;
            int totalClusters = clusterDemand.Length;

            for (int i = 0; i < vehiculesNumber; i++)
            {
                vehiculeCapacity[i] = capacity;
                clusterRouteForVehicule[i] = new List<int>();
            }

            int[] indexSortedClusterDemand = arraySortedByIndex(clusterDemand);           

            while (totalClusterVisited < totalClusters)
            {
                List<int> RCL = new List<int>();
                for (int i = 0; i < clusterDemand.Length; i++)
                {
                    int minDemandIndex = indexSortedClusterDemand[i];
                    if (!visitedCluster[minDemandIndex])
                    {
                        RCL.Add(minDemandIndex);
                    }
                    if (RCL.Count == rclsize) break;
                }

                Random rnd = new Random();
                int clusterRndIndex = rnd.Next(0, RCL.Count);
                int clusterRndSelected = RCL[clusterRndIndex];
                int maxRndVehiculeIt = 0;
                while (true)
                {
                    int vehiculeRndIndex = rnd.Next(0, vehiculesNumber);
                    if (vehiculeCapacity[vehiculeRndIndex] - clusterDemand[clusterRndSelected] >= 0)
                    {
                        vehiculeCapacity[vehiculeRndIndex] -= clusterDemand[clusterRndSelected];
                        clusterRouteForVehicule[vehiculeRndIndex].Add(clusterRndSelected);
                        visitedCluster[clusterRndSelected] = true;
                        totalDemand -= clusterDemand[clusterRndSelected];
                        totalClusterVisited++;
                        break;
                    }
                    maxRndVehiculeIt++;
                    if (maxRndVehiculeIt > 100)
                    {
                        //Logger.GetInstance().logLine("----BEST FIT----");
                        return assignVehiculesBestFitAlgorithm(clusterDemand, vehiculesNumber, capacity);
                    }
                }                         

            }
            //Logger.GetInstance().logLine("----NO BEST FIT----");
            return clusterRouteForVehicule;
        }

        static private Tuple<int, int, double> selectNextClusterAndNode(int actualNode, int actualCluster, List<int> clusterToVisit, double[][] nodesMatrixDistance, bool[][] clusterMatrix)
        {
            return selectNextClusterAndNodeGreedyRandomized(actualNode, actualCluster, clusterToVisit, nodesMatrixDistance, clusterMatrix);
        }
        static private Tuple<int, int, double> selectNextClusterAndNodeGreedyRandomized(int actualNode, int actualCluster, List<int> clusterToVisit, double[][] nodesMatrixDistance, bool[][] clustersMatrix, int rclsize = 2)
        {
            Tuple<int, int, double>[] RCL = new Tuple<int, int, double>[rclsize];
            for (int nextCluster = 0; nextCluster < clustersMatrix.Length; nextCluster++)
            {
                if (clusterToVisit.Contains(nextCluster))
                {
                    for (int closeNode = 0; closeNode < nodesMatrixDistance.Length; closeNode++)
                    {
                        if (clustersMatrix[nextCluster][closeNode] && actualNode != closeNode)
                        {
                            Tuple<int, int, double> nextPossibleNode = new Tuple<int, int, double>(closeNode, nextCluster, nodesMatrixDistance[closeNode][actualNode]);
                            insertIntoRCL(nextPossibleNode, RCL);
                        }
                    }
                }
            }
            Random rnd = new Random();
            int notNullPositions = countNotNullPositions(RCL);
            int rndIndex = rnd.Next(0, notNullPositions);
            return RCL[rndIndex];
        }

        static private Tuple<List<int>, double> calculateIntraClusterTravel(int actualNode, int actualCluster, int[] nodesOnCluster, double[][] nodesMatrixDistance)
        {
            return calculateIntraClusterTravelGreedyRandomized(actualNode, actualCluster, nodesOnCluster, nodesMatrixDistance);
        }
        static private Tuple<List<int>, double> calculateIntraClusterTravelGreedyRandomized(int actualNode, int actualCluster, int[] nodesOnCluster, double[][] nodesMatrixDistance, int rclsize = 2)
        {
            double totalDistance = 0;
            List<int> intraClusterTravel = new List<int>();
            intraClusterTravel.Add(actualNode);

            while (intraClusterTravel.Count < nodesOnCluster.Length)
            {
                Tuple<int, int, double>[] RCL = new NodeTupleDistance[rclsize];
                for (int nextNodeIdx = 0; nextNodeIdx < nodesOnCluster.Length; nextNodeIdx++)
                {
                    int nextNode = nodesOnCluster[nextNodeIdx];
                    if (!intraClusterTravel.Contains(nextNode))
                    {
                        double distance = nodesMatrixDistance[actualNode][nextNode];
                        NodeTupleDistance nextTupleNode = new NodeTupleDistance(nextNode, actualCluster, distance);
                        insertIntoRCL(nextTupleNode, RCL);
                    }
                }
                Random rnd = new Random();
                int notNullPositions = countNotNullPositions(RCL);
                int rndIndex = rnd.Next(0, notNullPositions);
                actualNode = RCL[rndIndex].Item1;
                totalDistance += RCL[rndIndex].Item3;
                intraClusterTravel.Add(actualNode);                
            }

            return new Tuple<List<int>, double>(intraClusterTravel, totalDistance);
        }

        // Return a bi-dimensional matrix where M[i][j] is true if cluster i contains node j
        static private bool[][] clustersMatrix(int[][] clusters, int nodesNumber)
        {
            bool[][] ret = new bool[clusters.Length][];
            for(int i = 0; i < clusters.Length; i++)
            {
                ret[i] = new bool[nodesNumber];
                for (int j = 0; j < nodesNumber; j++)
                {
                    ret[i][j] = clusters[i].Contains(j);
                }
            }
            return ret;
        }
        
        // Return a node's distance matrix
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

        // Verify if the clusters can be served by the sum of the vechicules capacity
        static private bool verifyClustersDemand(CluVRPInstance instance)
        {
            int availableCapacity = instance.vehicules() * instance.capacity();
            int[] clusterDemand = instance.clusters_demand();
            return availableCapacity >= clusterDemand.Sum();
        }
                      
        // Distance function
        static public double distance(double x1, double y1, double x2, double y2)
        {
            return Math.Sqrt(Math.Pow(x1-x2, 2) + Math.Pow(y1-y2, 2));
        }

        // Insert Node into candidate list
        static private void insertIntoRCL(Tuple<int, int, double> nextPossibleNode, Tuple<int, int, double>[] CL)
        {

            int bestIndex = -1;
            for(int i = 0; i < CL.Length; i++)
            {
                if (CL[i] == null)
                {
                    bestIndex = i;
                    break;
                }
                else if(CL[i].Item3 > nextPossibleNode.Item3)
                {
                    bestIndex = i;
                }                           
            }
            if(bestIndex!=-1)
            {
                CL[bestIndex] = nextPossibleNode;
            }
        } 

        // Return the last not null position on array
        static private int countNotNullPositions(Object[] arr)
        {
            int idx = arr.Length;
            for(int i = 0; i < arr.Length; i++)
            {
                if (arr[i] == null)
                {
                    idx = i;
                    break;
                }
            }
            return idx;
        }
        
        // Return an array with the index of the input array sorted
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
