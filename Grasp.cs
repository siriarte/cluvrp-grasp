using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NodeTupleDistance = System.Tuple<int, int, double>;

namespace CluVRP_GRASP
{

    class Grasp
    {
        static public double ConstructGreedySolution(CluVRPInstance instance, int iterationsForBestSolution = 10)
        {
            double[][] nodesMatrixDistance = calculateNodesMatrixDistance(instance);
            bool[][] clustersMatrix = Grasp.calculateClustersMatrix(instance.clusters(), instance.dimension());
            CluVRPSolution bestCluVRPSolution = new CluVRPSolution(null, null, double.MaxValue, null);
            double totalDistance;

            if (!verifyClustersDemand(instance))
            {
                Logger.GetInstance().logLine("Capacity demanded can't be served by the vehicules");
            }


            for (int iteration = 0; iteration < iterationsForBestSolution; iteration++)
            {

                totalDistance = 0;
                List<int>[] nodesRoute = new List<int>[instance.clusters().Length];
                List<int>[] clustersRoute = new List<int>[instance.vehicules()];

                List<int>[] vehiculeAssignation = assignVehicules(instance);
                int[] vehiculeFreeSpace = calculateFreeSpace(vehiculeAssignation, instance.capacity(), instance.clusters_demand());


                for (int vehiculeIndex = 0; vehiculeIndex < vehiculeAssignation.Length; vehiculeIndex++)
                {
                    int actualNode = 0;
                    int actualCluster = 0;
                    nodesRoute[actualCluster] = new List<int>();
                    nodesRoute[actualCluster].Add(actualNode);

                    List<int> clustersToVisit = vehiculeAssignation[vehiculeIndex];
                    int numberOfClusterToVisit = clustersToVisit.Count;
                    clustersRoute[vehiculeIndex] = new List<int>();

                    for (int clusterIt = 0; clusterIt + 1 < numberOfClusterToVisit; clusterIt++)
                    {
                        clustersRoute[vehiculeIndex].Add(actualCluster);
                        clustersToVisit.Remove(actualCluster);

                        if (clustersToVisit.Count != 0)
                        {
                            Tuple<int, int, double> nextClusterAndNode = selectNextClusterAndNode(actualNode, actualCluster,
                                clustersToVisit, nodesMatrixDistance, clustersMatrix);
                            actualNode = nextClusterAndNode.Item1;
                            actualCluster = nextClusterAndNode.Item2;
                            totalDistance += nextClusterAndNode.Item3;
                        }


                        Tuple<List<int>, double> intraClusterTravel = calculateIntraClusterTravel(actualNode, actualCluster,
                            instance.clusters()[actualCluster], nodesMatrixDistance);

                        totalDistance += intraClusterTravel.Item2;
                        actualNode = intraClusterTravel.Item1[intraClusterTravel.Item1.Count - 1];
                        nodesRoute[actualCluster] = intraClusterTravel.Item1;
                    }

                    totalDistance += nodesMatrixDistance[actualNode][0];
                    clustersRoute[vehiculeIndex].Add(actualCluster);
                    clustersRoute[vehiculeIndex].Add(0);
                }


                CluVRPSolution solution = new CluVRPSolution(clustersRoute, nodesRoute, totalDistance, vehiculeFreeSpace);
                localSearchs(solution, instance, nodesMatrixDistance, vehiculeFreeSpace);

                // Update best solution
                if (solution.totalDistance < bestCluVRPSolution.totalDistance)
                {
                    bestCluVRPSolution.totalDistance = solution.totalDistance;
                    bestCluVRPSolution.nodesRoute = nodesRoute;
                    bestCluVRPSolution.clustersRoute = clustersRoute;
                }
            }

            return bestCluVRPSolution.totalDistance;
        }

        static public void localSearchs(CluVRPSolution solution, CluVRPInstance instance, double[][] nodesMatrixDistance, int[] vehiculeFreeSpace)
        {
            swapInterCluster(solution, instance, nodesMatrixDistance, vehiculeFreeSpace);
            swapIntraCluster(solution, nodesMatrixDistance);
         }

        static public int[] calculateFreeSpace(List<int>[] vechiculeRoute, int capacity, int[] clusterDemand)
        {
            int[] res = new int[vechiculeRoute.Length];
            for (int i = 0; i < vechiculeRoute.Length; i++)
            {
                res[i] = capacity;
                for (int j = 0; j < vechiculeRoute[i].Count; j++)
                {
                    res[i] -= clusterDemand[vechiculeRoute[i][j]];
                }
            }
            return res;
        }

        static public void swapIntraCluster(CluVRPSolution solution, double[][] nodesMatrixDistance)
        {

            for (int vehiculeIndex = 0; vehiculeIndex < solution.clustersRoute.Length; vehiculeIndex++)
            {
                int clusterSize = solution.clustersRoute[vehiculeIndex].Count;

                for (int clusterIt1 = 1; clusterIt1 < clusterSize; clusterIt1++)
                {
                    for (int clusterIt2 = clusterIt1 + 1; clusterIt2 < clusterSize; clusterIt2++)
                    {
                        Swap(solution.clustersRoute[vehiculeIndex], clusterIt1, clusterIt2);
                        double newDistance = calculateRouteDistance(solution.nodesRoute, solution.clustersRoute, nodesMatrixDistance);

                        if (newDistance > solution.totalDistance || 
                            solution.clustersRoute[vehiculeIndex][clusterIt1] == 0 || 
                            solution.clustersRoute[vehiculeIndex][clusterIt2] == 0)
                        {
                            Swap(solution.clustersRoute[vehiculeIndex], clusterIt2, clusterIt1);
                        }
                        else
                        {
                            solution.totalDistance = newDistance;
                        }

                    }
                }
            }
        }

        static public void swapInterCluster(CluVRPSolution solution, CluVRPInstance instance, double[][] nodesMatrixDistance, int[] vehiculeFreeSpace)
        {            
            for(int vechiculeIt1 = 0; vechiculeIt1 < solution.clustersRoute.Length; vechiculeIt1++)
            {
                int clusterSize1 = solution.clustersRoute[vechiculeIt1].Count;

                for (int vechiculeIt2 = vechiculeIt1 + 1; vechiculeIt2 < solution.clustersRoute.Length; vechiculeIt2++)
                {
                    int clusterSize2 = solution.clustersRoute[vechiculeIt2].Count;

                    for (int clusterIt1 = 1; clusterIt1 < clusterSize2; clusterIt1++)
                    {
                        for (int clusterIt2 = clusterIt1 + 1; clusterIt2  < clusterSize2; clusterIt2++)
                        {
                            if(swapCluster(solution.clustersRoute, vechiculeIt1, vechiculeIt2, clusterIt1, clusterIt2, instance, vehiculeFreeSpace))
                            {
                                double newDistance = calculateRouteDistance(solution.nodesRoute, solution.clustersRoute, nodesMatrixDistance);
                                if (newDistance > solution.totalDistance)
                                {
                                    swapCluster(solution.clustersRoute, vechiculeIt1, vechiculeIt2, clusterIt1, clusterIt2, instance, vehiculeFreeSpace);
                                }
                                else
                                {
                                    solution.totalDistance = newDistance;
                                }
                            }

                        }
                    }
                }                    
            }                    
        }

        static bool swapCluster(List<int>[] clustersRoute, int vehiculeIdx1, int vehiculeIdx2, int clusterIdx1, int clusterIdx2, CluVRPInstance instance, int[] vehiculeFreeSpace)
        {
            int cluster1 = clustersRoute[vehiculeIdx1][clusterIdx1];
            int cluster2 = clustersRoute[vehiculeIdx2][clusterIdx2];

            int clu1Demand = instance.clusters_demand()[cluster1];
            int clu2Demand = instance.clusters_demand()[cluster2];
            
            if( vehiculeFreeSpace[vehiculeIdx1] + clu1Demand - clu2Demand > 0 &&
                vehiculeFreeSpace[vehiculeIdx2] + clu2Demand - clu1Demand > 0 
              )
            {
                clustersRoute[vehiculeIdx1][clusterIdx1] = cluster2;
                clustersRoute[vehiculeIdx2][clusterIdx2] = cluster1;
                vehiculeFreeSpace[vehiculeIdx1] = vehiculeFreeSpace[vehiculeIdx1] + clu1Demand - clu2Demand;
                vehiculeFreeSpace[vehiculeIdx2] = vehiculeFreeSpace[vehiculeIdx2] + clu2Demand - clu1Demand;
                return true;
            }
            return false;
        }

        static public double calculateRouteDistance(List<int>[] nodesRoute, List<int>[] clustersRoute, double[][] nodesMatrixDistance)
        {
            double totalDistance = 0;
            int lastNode;
            int nextNode;
            int actualCluster = 0;
            int nextCluster;

            for(int vehiculeIndex = 0; vehiculeIndex < clustersRoute.Length; vehiculeIndex++)
            {
                for(int clusterIndex = 0; clusterIndex < clustersRoute[vehiculeIndex].Count; clusterIndex++)
                {

                    actualCluster = clustersRoute[vehiculeIndex][clusterIndex];

                    // Sum intracluster distance
                    for (int k = 0; k + 1 < nodesRoute[actualCluster].Count; k++)
                    {
                        lastNode = nodesRoute[actualCluster][k];
                        nextNode = nodesRoute[actualCluster][k + 1];
                        totalDistance += nodesMatrixDistance[lastNode][nextNode];
                    }

                    // Jump to next cluster node
                    if(clusterIndex + 1 < clustersRoute[vehiculeIndex].Count)
                    {
                        nextCluster = clustersRoute[vehiculeIndex][clusterIndex+1];
                        lastNode = nodesRoute[actualCluster][nodesRoute[actualCluster].Count - 1];
                        nextNode = nodesRoute[nextCluster][0];
                        totalDistance += nodesMatrixDistance[lastNode][nextNode];
                    }
                }
          }
            return totalDistance;
        }


        // Return an assignation of nodes to vehicules
        static private List<int>[] assignVehicules(CluVRPInstance instance, int baseNode = 0)
        {
            int[] clusterDemand = instance.clusters_demand();
            int vehiculesNumber = instance.vehicules();
            int capacity = instance.capacity();
            return assignVehiculesBestFitAlgorithm(clusterDemand, vehiculesNumber, capacity, baseNode);
        }

        // Return a vehicule assignation using best fit algorithm
        // https://www.geeksforgeeks.org/bin-packing-problem-minimize-number-of-used-bins/
        static private List<int>[] assignVehiculesBestFitAlgorithm(int[] clusterDemand, int vehiculesNumber, int capacity, int baseNode)
        {
            List<int>[] clusterRouteForVehicule = new List<int>[vehiculesNumber];
            int[] vechiculeRemSpace = new int[vehiculesNumber];
            for (int i = 0; i < vehiculesNumber; i++)
            {
                //vechiculeRemSpace[i] = capacity;
                clusterRouteForVehicule[i] = new List<int>();
                clusterRouteForVehicule[i].Add(baseNode);
            }

            int res = 0;

            for (int i = 1; i < clusterDemand.Length; i++)
            {
                int j;
                int min = capacity + 1, bi = 0;

                for (j = 0; j < res; j++)
                {
                    if (vechiculeRemSpace[j] >= clusterDemand[i] &&
                        vechiculeRemSpace[j] - clusterDemand[i] < min)
                    {
                        bi = j;
                        min = vechiculeRemSpace[j] - clusterDemand[i];
                    }
                }

                if (min == capacity + 1)
                {
                    vechiculeRemSpace[res] = capacity - clusterDemand[i];
                    clusterRouteForVehicule[res].Add(i);
                    res++;
                } else
                {
                    vechiculeRemSpace[bi] -= clusterDemand[i];
                    clusterRouteForVehicule[bi].Add(i);
                }
            }

            int clusterVisited = 0;
            for(int i = 0; i < clusterRouteForVehicule.Length; i++)
            {
                clusterVisited += clusterRouteForVehicule[i].Count;
            }

            if(clusterVisited - 1 < clusterDemand.Length)
            {
                Logger.GetInstance().logLine("Hay clusters sin visitar");
            }


            return clusterRouteForVehicule;
        }  
        static private List<int>[] assignVehiculesFirstFitAlgorithm(int[] clusterDemand, int vehiculesNumber, int capacity, int baseNode)
        {
            List<int>[] clusterRouteForVehicule = new List<int>[vehiculesNumber];
            int[] vehiculeCapacity = new int[vehiculesNumber];
            for (int i = 0; i < vehiculesNumber; i++)
            {
                vehiculeCapacity[i] = capacity;
                clusterRouteForVehicule[i] = new List<int>();

                // add base
                clusterRouteForVehicule[i].Add(baseNode);
            }

            int[] indexSortedClustedDemand = arraySortedByIndex(clusterDemand);
            for (int i = 0; i < clusterDemand.Length; i++)
            {
                int minCapacityIndex = indexSortedClustedDemand[i];
                if (minCapacityIndex != baseNode)
                {
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
            }
            
            return clusterRouteForVehicule;
        }
        static private List<int>[] assignVehiculesBestFitRandomizedAlgorithm(int[] clusterDemand, int vehiculesNumber, int capacity, int baseNode, int rclsize = 2)
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
                        return assignVehiculesFirstFitAlgorithm(clusterDemand, vehiculesNumber, capacity, baseNode);
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
        static private bool[][] calculateClustersMatrix(int[][] clusters, int nodesNumber)
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
            //return 1;
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
                    if (arr[ret[j]] >= arr[ret[i]])
                    {
                        int temp = ret[i];
                        ret[i] = ret[j];
                        ret[j] = temp;
                    }
                }
            }
            return ret;
        }

        // Swap
        public static void Swap<T>(IList<T> list, int indexA, int indexB)
        {
            T tmp = list[indexA];
            list[indexA] = list[indexB];
            list[indexB] = tmp;
        }

    }
}
