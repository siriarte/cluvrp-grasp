using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ClusterSolution = System.Tuple<System.Collections.IList[], double>;

/*
 * Grasp for CluVRP at cluster-level. 
 * 
 * M <- CalculateClusterDistance()
 * BestSolution <-*
 * While(StopCondition)
 *   Solution <- RandomizedGreedySolution()
 *   NewSolution <- LocalSearch(S)
 *   if( NewSolution isBetterThan BestSolution) 
 *      BestSolution = NewSolution
 * return bestSolution
 * 
 * 
 */

namespace cluvrp_grasp
{
    class ClusterLevelSolution
    {

        // Public variables
        public CluVRPInstance instance;
        public double[][] customersDistanceMatrix;
        public double[][] clustersDistanceMatrix;
        public ClusterSolution bestSolution;
        public List<int>[] solutionRoute;
        public int graspIterations;
        
        // Constructor
        public ClusterLevelSolution(CluVRPInstance instance)
        {
            // Set instance 
            this.instance = instance;

            // Init best solution
            this.bestSolution = new ClusterSolution(new List<int>[0], double.MaxValue);

        }

        /*
         * Calculate the intercluster distances 
         * Use the shortest edge between two clusters as an approximation for the inter-cluster distance
         *           
         */
        public void calculateClusterDistanceMatrix()
        {
            // Variables from instances 
            int[][] clusters = this.instance.clusters();
            int numberOfClusters = clusters.Length;

            // Matrix to return
            double[][] clustersDistanceMatrix = new double[numberOfClusters][];

            for (int clusterIt1 = 0; clusterIt1 < numberOfClusters; clusterIt1++)
            {
                // Create double[]
                clustersDistanceMatrix[clusterIt1] = new double[numberOfClusters];

                // First cluster to iterate
                int[] cluster1 = clusters[clusterIt1];

                for (int clusterIt2 = 0; clusterIt2 < numberOfClusters; clusterIt2++)
                {
                    // Second cluster to iterate
                    int[] cluster2 = clusters[clusterIt2];

                    // Calculate best distance between 2 cluster
                    double distance = bestDistanceBetween2Clusters(cluster1, cluster2);

                    // Update matrix value
                    clustersDistanceMatrix[clusterIt1][clusterIt2] = distance;
                }
            }
            this.clustersDistanceMatrix = clustersDistanceMatrix;
        }

        /*
         * 
         * Calculte min distance between two clusters
         * We use the shortest edge between two clusters as an approximation 
         * for the inter-cluster distance
         *
         */
        private double bestDistanceBetween2Clusters(int[] cluster1, int[] cluster2)
        {

            // Define min distance
            double minDistance = double.MaxValue;

            // For each customer in cluster 1
            for (int i = 0; i < cluster1.Length; i++)
            {
                // Customer 1
                int customer1 = cluster1[i];

                // For each customer in cluster 2
                for(int j = 0; j < cluster2.Length; j++)
                {
                    // Customer 2
                    int customer2 = cluster2[j];

                    // Get distance between customer 1 and customer 2
                    double distance = this.customersDistanceMatrix[customer1][customer2];

                    // Update min distance
                    if(distance < minDistance)
                    {
                        minDistance = distance;
                    }
                }
            }

            // Return min distance
            return minDistance;
        }

        /*
         * 
         * Grasp():
	     * M <- calculateClusterDistance
	     * BestSolution = 0
	     * While(StopCondition)
		 *      Solution <- ConstructGreedySolution()
		 *      NewSolution <- LocalSearch(solution)
		 *      if NewSolution isBetterThan BestSolution
         *          BestSolution = NewSolution
	     * return BestSolution
         *
         */
        public void Grasp(int interations = 100, double alphaCapacity = 0.5, double alphaDistance = 0.5)
        {
            
            // Calculate customers distance matrix 
            this.customersDistanceMatrix = functions.customersDistanceMatrix(instance);

            // Calculate cluster distance matrix 
            this.calculateClusterDistanceMatrix();
            
            // Set iterator 
            int iterator = 0;
            
            // Main Cycle
            while(iterator < interations)
            {
                // Construct a Greedy Randomized Solution for alpha parameter
                ClusterSolution solution = constructGreedyRandomizedSolution(alphaCapacity, alphaDistance);
            }

            return;
        }
        
        /*
         * Construct an Greedy Randomized solution
         * 
         * constructGreedyRandomizedSolution():
         * solution = empty
         * while(noCompleteSolution)
         *      BuildRCL()
         *      s = SelectRandomElement(RCL)
         *      solution = solution U (s)
         *      AdaptGreedySolution(s)
         * 
         */
        private ClusterSolution constructGreedyRandomizedSolution(double alphaDemand, double alphaDistance)
        {
            // Init variables
            int numberOfVehicules = instance.vehicules();
            int vehiculeCapacity = instance.capacity();
            int[][] clusters = instance.clusters();
            int numbersOfClusters = clusters.Length;
            int[] clustersDemand = instance.clusters_demand();
            int[] vechiculeRemSpace = new int[numberOfVehicules];
            List<int> clustersToVisit = new List<int>();

            // Default solution
            ClusterSolution solution = new ClusterSolution(new List<int>[numberOfVehicules], double.MaxValue);
            
            // Create route list and add depot cluster (0) to vehicules route
            for (int i = 0; i < numberOfVehicules; i++)
            {
                solution.Item1[i] = new List<int>();
                solution.Item1[i].Add(0);
            }

            // Set clusters to visit (all except depot, cluster 0)
            for (int i = 1; i < numbersOfClusters; i++)
            {
                clustersToVisit.Add(i);
            }
            
            // Main Cycle 
            // While exists clusters to visit 
            for(int i = 0; i < numbersOfClusters; i++)
            {
                List<int> clusterByDemandRCL = buildClusterByDemandRCL(clustersToVisit, clustersDemand, alphaDemand);
                int clusterSelected = selectFromRCL(clusterByDemandRCL);
                List<int> vehiculeBydistanceRCL = buildVehiculeByDistanceRCL(vechiculeRemSpace, clusterSelected, alphaDistance);
                int vehiculeSelected = selectFromRCL(vehiculeBydistanceRCL);
            }

            return solution;
        }

        /*
         * 
         *  Build RCL with the criteria of minimal demand of the cluster         
         * 
         */
        private List<int> buildClusterByDemandRCL(List<int> clustersToVisit, int[] clustersDemand, double alphaDemand)
        {
            // Init variables
            List<int> RCL = new List<int>();
            List<int> clustersSortedByDemand = sortClustersByDemand(clustersToVisit, clustersDemand);
            int maxDemand = clustersDemand.Max();
            int minDemand = clustersDemand.Min();

            // Define the RCL demand condition
            double rclCondition = minDemand + alphaDemand * (maxDemand - minDemand);

            // Iterate all cluster to visit
            for (int i = 0; i < clustersToVisit.Count; i++)
            {
                int cluster = clustersToVisit[i];

                // Verify condition and add to rcl list
                if(clustersDemand[cluster] <= rclCondition)
                {
                    RCL.Add(cluster);
                }
            }

            // Return the RCL
            return RCL;            
        }

        /*
         *
         * Return a cluster list sorted by demand 
         * 
         */
        private List<int> sortClustersByDemand(List<int> clustersToVisit, int[] clustersDemand)
        {
            // Init variables
            int listSize = clustersToVisit.Count;
            List<int> sortedList = new List<int>(clustersToVisit);

            
            // Iterate and swap  
            for(int i = 0; i < listSize; i++)
            {
                for(int j = i + 1; j < listSize; j++)
                {
                    if(clustersDemand[sortedList[i]] > clustersDemand[sortedList[j]])
                    {
                        functions.Swap(sortedList, i, j);
                    }
                }
            }

            // return sorted list
            return sortedList;
        }

        private List<int> buildVehiculeByDistanceRCL(int[] vechiculeRemSpace, int clusterSelected, double alphaDistance)
        {
            throw new NotImplementedException();
        }

        /*
         * 
         * Select a element from a RCL list with random criteria
         * 
         */
        private int selectFromRCL(List<int> list)
        {
            Random rnd = new Random();
            int rndIndex = rnd.Next(0, list.Count);
            return list[rndIndex];
        }

    }
}
