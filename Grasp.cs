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
 
            double[][] nodesMatrixDistance = calculateNodesMatrixDistance(instance);
            bool[][] clustersMatrix = Grasp.clustersMatrix(instance.clusters(), instance.dimension());
            double totalDistance = 0;
            NodeTupleDistance[][][] interClusterDistancesv2 = calculateInterClusterMatrixDistanceTupleVersion(instance);
            double[][][][] interClusterDistances = calculateInterClusterMatrixDistance(instance);
            List<int>[] vehiculeAssignation = assignVehicules(instance, interClusterDistancesv2);

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

                    List<int> clustersToVisit = vehiculeAssignation[vehiculeIndex];
                    for(int clusterIt = 0; clusterIt < clustersToVisit.Count; clusterIt++)
                    {
                        clustersToVisit.Remove(actualCluster);
                        Tuple<int, int, double> nextClusterAndNode = selectNextClusterAndNodeRandomizedGreedy(actualNode, actualCluster, clustersToVisit, nodesMatrixDistance, clustersMatrix);
                        actualNode = nextClusterAndNode.Item1;
                        actualCluster = nextClusterAndNode.Item2;
                        totalDistance += nextClusterAndNode.Item3;

                        Tuple<List<int>, double> intraClusterTravel = calculateIntraClusterTravelRandomizedGreedy(actualNode, actualCluster, instance.clusters()[actualCluster], nodesMatrixDistance);
                        totalDistance += intraClusterTravel.Item2;
                        actualNode = intraClusterTravel.Item1[intraClusterTravel.Item1.Count - 1];
                        //Logger.GetInstance().logLine(String.Join(",", intraClusterTravel.Item1));
                    }
                    totalDistance += nodesMatrixDistance[actualNode][0];
                }

            }
            return totalDistance;
        }

        static private Tuple<int, int, double> selectNextClusterAndNode(int actualNode, int actualCluster, List<int> clusterToVisit, double[][] nodesMatrixDistance, bool[][] clusterMatrix)
        {
            return selectNextClusterAndNodeGreedy(actualNode, actualCluster, clusterToVisit, nodesMatrixDistance, clusterMatrix);
        }
        
        static private Tuple<List<int>, double> calculateIntraClusterTravel(int actualNode, int actualCluster, int[] nodesOnCluster, double[][] nodesMatrixDistance)
        {
            return calculateIntraClusterTravelGreedy(actualNode, actualCluster, nodesOnCluster, nodesMatrixDistance);
        }

        static private Tuple<int, int, double> selectNextClusterAndNodeGreedy(int actualNode, int actualCluster, List<int> clusterToVisit, double[][] nodesMatrixDistance, bool[][] clustersMatrix)
        {
            Tuple<int, int, double> bestTravel = new Tuple<int, int, double>(actualNode, actualCluster, double.MaxValue);
            for (int nextCluster = 0; nextCluster < clustersMatrix.Length; nextCluster++)
            {
                if (clusterToVisit.Contains(nextCluster))
                {
                    for (int closeNode = 0; closeNode < nodesMatrixDistance.Length; closeNode++)
                    {
                        if (clustersMatrix[nextCluster][closeNode] && actualNode != closeNode && nodesMatrixDistance[closeNode][actualNode] < bestTravel.Item3)
                        {
                            bestTravel = new Tuple<int, int, double>(closeNode, nextCluster, nodesMatrixDistance[closeNode][actualNode]);
                        }
                    }
                }
            }
            return bestTravel;
        }

        static private Tuple<int, int, double> selectNextClusterAndNodeRandomizedGreedy(int actualNode, int actualCluster, List<int> clusterToVisit, double[][] nodesMatrixDistance, bool[][] clustersMatrix, int clsize = 3)
        {
            Tuple < int, int, double>[] CL = new Tuple<int, int, double>[clsize];
            for (int nextCluster = 0; nextCluster < clustersMatrix.Length; nextCluster++)
            {
                if (clusterToVisit.Contains(nextCluster))
                {
                    for (int closeNode = 0; closeNode < nodesMatrixDistance.Length; closeNode++)
                    {
                        if (clustersMatrix[nextCluster][closeNode] && actualNode != closeNode)
                        {
                            Tuple<int, int, double>  nextPossibleNode = new Tuple<int, int, double>(closeNode, nextCluster, nodesMatrixDistance[closeNode][actualNode]);
                            insertIntoCL(nextPossibleNode, CL);                           
                        }
                    }
                }
            }
            Random rnd = new Random();
            int notNullPositions = countNotNullPositions(CL);
            int rndIndex = rnd.Next(0, notNullPositions);
            return CL[rndIndex];
        }


        static private Tuple<List<int>, double> calculateIntraClusterTravelGreedy(int actualNode, int actualCluster, int[] nodesOnCluster, double[][] nodesMatrixDistance)
        {
            double totalDistance = 0;
            List<int> intraClusterTravel = new List<int>();
            intraClusterTravel.Add(actualNode);

            while (intraClusterTravel.Count < nodesOnCluster.Length)
            {
                double bestDistance = double.MaxValue;
                int nextBestNode = 0;
                for (int nextNodeIdx = 0; nextNodeIdx < nodesOnCluster.Length; nextNodeIdx++)
                {
                    int nextNode = nodesOnCluster[nextNodeIdx];
                    if(nodesMatrixDistance[actualNode][nextNode] < bestDistance && !intraClusterTravel.Contains(nextNode))
                    {
                        bestDistance = nodesMatrixDistance[actualNode][nextNode];
                        nextBestNode = nodesOnCluster[nextNodeIdx];
                    }
                }
                intraClusterTravel.Add(nextBestNode);
                actualNode = nextBestNode;
                totalDistance += bestDistance;
            }

            return new Tuple<List<int>, double>(intraClusterTravel, totalDistance);
        }


        static private Tuple<List<int>, double> calculateIntraClusterTravelRandomizedGreedy(int actualNode, int actualCluster, int[] nodesOnCluster, double[][] nodesMatrixDistance, int clsize = 3)
        {
            double totalDistance = 0;
            List<int> intraClusterTravel = new List<int>();
            intraClusterTravel.Add(actualNode);

            while (intraClusterTravel.Count < nodesOnCluster.Length)
            {
                Tuple<int, int, double>[] CL = new NodeTupleDistance[clsize];
                for (int nextNodeIdx = 0; nextNodeIdx < nodesOnCluster.Length; nextNodeIdx++)
                {
                    int nextNode = nodesOnCluster[nextNodeIdx];
                    if (!intraClusterTravel.Contains(nextNode))
                    {
                        double distance = nodesMatrixDistance[actualNode][nextNode];
                        NodeTupleDistance nextTupleNode = new NodeTupleDistance(nextNode, actualCluster, distance);
                        insertIntoCL(nextTupleNode, CL);
                    }
                }
                Random rnd = new Random();
                int notNullPositions = countNotNullPositions(CL);
                int rndIndex = rnd.Next(0, notNullPositions);
                actualNode = CL[rndIndex].Item1;
                totalDistance += CL[rndIndex].Item3;
                intraClusterTravel.Add(actualNode);                
            }

            return new Tuple<List<int>, double>(intraClusterTravel, totalDistance);
        }


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

        //Insert Node into candidate list
        static private void insertIntoCL(Tuple<int, int, double> nextPossibleNode, Tuple<int, int, double>[] CL)
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

        //
        // DEPRECATED
        //     

        // Create a inter-clusters distance array 
        // [i][j] is the array of tubles <node1, node2, distance> of distances between all nodes
        // on the i-cluster and j-cluster
        static private NodeTupleDistance[][][] calculateInterClusterMatrixDistanceTupleVersion(CluVRPInstance instance)
        {
            double[][] nodesDistanceMatrix = calculateNodesMatrixDistance(instance);
            int clustersNumber = instance.clusters_demand().Length;
            NodeTupleDistance[][][] clusterDistanceMatrix = new Tuple<int, int, double>[clustersNumber][][];
            int[][] clusters = instance.clusters();

            for (int i = 0; i < clustersNumber; i++)
            {
                clusterDistanceMatrix[i] = new Tuple<int, int, double>[clustersNumber][];
                for (int j = 0; j < clusters.Length; j++)
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

        static private double[][][][] calculateInterClusterMatrixDistance(CluVRPInstance instance)
        {
            double[][] nodesDistanceMatrix = calculateNodesMatrixDistance(instance);
            int nodesNumber = nodesDistanceMatrix.Length;
            int clustersNumber = instance.clusters_demand().Length;
            double[][][][] interClusterDistance = new double[clustersNumber][][][];
            int[][] clusters = instance.clusters();

            for (int clusterAIt = 0; clusterAIt < clustersNumber; clusterAIt++)
            {
                Console.WriteLine(clusterAIt);
                interClusterDistance[clusterAIt] = new double[clustersNumber][][];

                for (int clusterBIt = 0; clusterBIt < clustersNumber; clusterBIt++)
                {
                    interClusterDistance[clusterAIt][clusterBIt] = new double[nodesNumber][];

                    for (int nodeAIt = 0; nodeAIt < nodesNumber; nodeAIt++)
                    {
                        interClusterDistance[clusterAIt][clusterBIt][nodeAIt] = new double[nodesNumber];

                        if (clusters[clusterAIt].Contains(nodeAIt))
                        {
                            for (int nodeBIt = 0; nodeBIt < nodesNumber; nodeBIt++)
                            {
                                if (clusters[clusterBIt].Contains(nodeBIt))
                                {
                                    interClusterDistance[clusterAIt][clusterBIt][nodeAIt][nodeBIt] = nodesDistanceMatrix[nodeAIt][nodeBIt];
                                    //string result = String.Format("{0}\t{1}\t{2}\t{3}\t{4}", clusterAIt, clusterBIt, nodeAIt, nodeBIt, nodesDistanceMatrix[nodeAIt][nodeBIt]);
                                    //Logger.GetInstance().logLine(result);
                                }
                                else
                                {
                                    interClusterDistance[clusterAIt][clusterBIt][nodeAIt][nodeBIt] = double.MaxValue;
                                }
                            }

                        }
                        else
                        {
                            for (int nodeBIt = 0; nodeBIt < nodesNumber; nodeBIt++)
                            {
                                interClusterDistance[clusterAIt][clusterBIt][nodeAIt][nodeBIt] = double.MaxValue;

                            }
                        }
                    }

                }
            }
            return interClusterDistance;
        }

        static private Tuple<int, int, double> selectNextClusterAndNodeGreedyTupleVersion(int actualNode, int actualCluster, List<int> clusterToVisit, NodeTupleDistance[][][] interClusterDistances)
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

        static private Tuple<List<int>, double> calculateIntraClusterTravelGreedyTupleVersion(int actualNode, int actualCluster, int[] nodesOnCluster, NodeTupleDistance[][][] interClusterDistances)
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

    }
}
