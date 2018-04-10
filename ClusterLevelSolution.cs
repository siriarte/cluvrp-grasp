using System;
using System.Collections;
using System.Collections.Generic;
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
        
        /* 
         * 
         * Constructor
         * 
         */
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
        public void Grasp(int totalIterations = 100, double alphaCapacity = 0.5, double alphaDistance = 0.5)
        {
            
            // Calculate customers distance matrix 
            this.customersDistanceMatrix = Functions.customersDistanceMatrix(instance);

            // Calculate cluster distance matrix 
            this.calculateClusterDistanceMatrix();
            
            // Set iterator 
            int iterator = 0;
            
            // Main Cycle
            while(iterator < totalIterations)
            {
                // Construct a Greedy Randomized Solution for alpha parameter
                ClusterSolution solution = constructGreedyRandomizedSolution(alphaCapacity, alphaDistance);

                // Local search 
                this.localSearch(solution);

                // Update Best solution
                if(solution.Item2 < bestSolution.Item2)
                {
                    bestSolution = solution;
                }

                // Increace iterator
                iterator++;
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
            int numberOfVehicles = instance.vehicles();
            int vehicleCapacity = instance.capacity();
            int[][] clusters = instance.clusters();
            int numbersOfClusters = clusters.Length;
            int[] clustersDemand = instance.clusters_demand();
            int[] vehicleRemSpace = new int[numberOfVehicles];
            List<int> clustersToVisit = new List<int>();
            
            // Init solution route for vehicle
            List<int>[] clusterRouteForVehicle = new List<int>[numberOfVehicles];

            // Set vehicle remaning capacity AND
            // Init list of cluster that vehicle visits and add depot to the route
            for (int i = 0; i < numberOfVehicles; i++)
            {
                vehicleRemSpace[i] = vehicleCapacity;
                clusterRouteForVehicle[i] = new List<int>();
                clusterRouteForVehicle[i].Add(0);
            }

            // Set clusters to visit (all except depot, cluster 0)
            for (int i = 1; i < numbersOfClusters; i++)
            {
                clustersToVisit.Add(i);
            }

            // Main Cycle 
            // While exists clusters to visit 
            for(int i = 0; i + 1 < numbersOfClusters; i++)
            {
                // Create RCL for clusters by demand
                List<int> clusterByDemandRCL = buildClusterByDemandRCL(clustersToVisit, clustersDemand, alphaDemand);

                // Select cluster for RCL
                int clusterSelected = selectFromRCL(clusterByDemandRCL);

                // Create RCL for vehicle of clusterSeleted by distance (and bestFit capacity)
                List<int> vehicleBydistanceRCL = buildVehicleByDistanceRCL(clusterRouteForVehicle, vehicleCapacity, vehicleRemSpace, clusterSelected, clustersDemand[clusterSelected], alphaDistance);

                // Only add the cluster to the route of vehicle if were possible fit it
                if (vehicleBydistanceRCL.Count > 0)
                {
                    // Select vehicle from RCL 
                    int vehicleSelected = selectFromRCL(vehicleBydistanceRCL);

                    // Add cluster to vehicle route
                    clusterRouteForVehicle[vehicleSelected].Add(clusterSelected);

                    // Update vehicle remmaing space
                    vehicleRemSpace[vehicleSelected] -= clustersDemand[clusterSelected];

                    // Remove cluster of the list to visit
                    clustersToVisit.Remove(clusterSelected);
                }
            }

            // If there are clusters without vehicle
            if(clustersToVisit.Count != 0)
            {
                return new ClusterSolution(new List<int>[0], 0);
            }

            // Add depot as final cluster for all travels
            for(int i = 0; i < numberOfVehicles; i++)
            {
                clusterRouteForVehicle[i].Add(0);
            }

            // Calculte total inter-cluster distance
            double travelTotalDistance = this.calculateTotalTravelDistance(clusterRouteForVehicle);

            // Set solution
            ClusterSolution solution = new ClusterSolution(clusterRouteForVehicle, travelTotalDistance);

            // Return solution
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

            // Set min and max demand with the sorted list
            int firstCluster = clustersSortedByDemand[0];
            int lastCluster = clustersSortedByDemand[clustersSortedByDemand.Count - 1];
            int minDemand = clustersDemand[firstCluster];
            int maxDemand = clustersDemand[lastCluster];

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
                        Functions.Swap(sortedList, i, j);
                    }
                }
            }

            // return sorted list
            return sortedList;
        }

        /*
         * 
         *  Build RCL with the criteria of minimal distance bewteen next cluster 
         *  and last cluster on each vehicle      
         * 
         */
        private List<int> buildVehicleByDistanceRCL(IList[] clusterRouteForVehicle, int vehicleCapacity, int[] vechiculeRemSpace, int clusterSelected, int clusterDemand, double alphaDistance)
        {
            // Set variables
            List<int> RCL = new List<int>();
            int numberOfVehicles = vechiculeRemSpace.Length;
            int minCapacity = vehicleCapacity + 1;
            int bestIndex = 0;

            // Calculate max and min distance for RCL condition
            double minDistance = minClusterDistance(clusterRouteForVehicle, clusterSelected);
            double maxDistance = maxClusterDistance(clusterRouteForVehicle, clusterSelected);

            // Set RCL condition criteria
            double RCLCondition = minDistance + alphaDistance * (maxDistance - minDistance);
            
            // For each vehicle
            for (int j = 0; j < numberOfVehicles; j++)
            {

                // Calculate the efective distance beetwen the last cluster visited 
                // by vehicle j and clusterSelected
                int lastClusterVisited = (int)clusterRouteForVehicle[j][clusterRouteForVehicle[j].Count-1];
                double distanceBetweenClusters = this.clustersDistanceMatrix[lastClusterVisited][clusterSelected];

                // If there is space on vehicle j AND
                // The remaning space is less than minCapacity AND
                // the distance es acceptable for RCL condition
                if (vechiculeRemSpace[j] >= clusterDemand &&
                    vechiculeRemSpace[j] - clusterDemand < minCapacity)
                {
                    // Add the vehicle to RCL if is possible
                    if (distanceBetweenClusters <= RCLCondition)
                    {
                        RCL.Add(j);
                    }

                    // Update min capacity
                    minCapacity = vechiculeRemSpace[j] - clusterDemand;
                    bestIndex = j;
                }
                
             }
            // If RCL is empty insert the vehicle that not has the distante condition
            if (RCL.Count == 0)
            {
               RCL.Add(bestIndex);
            }
            
            // return rcl
            return RCL;
        }

        /*
         * 
         * Calculte the max distance between the last cluster 
         * visited (of all vehicles) and toCluster
         * 
         */
        private double maxClusterDistance(IList[] clusterRouteForVehicle, int toCluster)
        {
            double ret = double.MinValue;
            for(int i = 0; i < clusterRouteForVehicle.Length; i++)
            {
                int lastClusterIndex = clusterRouteForVehicle[i].Count - 1;
                int lastCluster = (int)clusterRouteForVehicle[i][lastClusterIndex];
                ret = Math.Max(ret, this.clustersDistanceMatrix[lastCluster][toCluster]);
            }
            return ret;
        }

        /*
         * 
         * Calculte the min distance between the last cluster 
         * visited (of all vehicles) and toCluster
         * 
         */
        private double minClusterDistance(IList[] clusterRouteForVehicle, int toCluster)
        {
            double ret = double.MaxValue;
            for (int i = 0; i < clusterRouteForVehicle.Length; i++)
            {
                int lastClusterIndex = clusterRouteForVehicle[i].Count - 1;
                int lastCluster = (int)clusterRouteForVehicle[i][lastClusterIndex];
                ret = Math.Min(ret, this.clustersDistanceMatrix[lastCluster][toCluster]);
            }
            return ret;
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
        
        /*
         *
         * Calculate the total distance of the cluster travel
         *
         */ 
        private double calculateTotalTravelDistance(List<int>[] travel)
        {
            // Set variables
            double totalDistance = 0;

            // Iterate each vehicle
            for (int vehicleNumber = 0; vehicleNumber < travel.Length; vehicleNumber++)
            {
                // Iterate each cluster on vehicle route
                for (int clusterIt = 0; clusterIt + 1 < travel[vehicleNumber].Count; clusterIt++)
                {
                    int fromCluster = travel[vehicleNumber][clusterIt];
                    int ToCluster = travel[vehicleNumber][clusterIt + 1];
                    totalDistance += this.clustersDistanceMatrix[fromCluster][ToCluster];
                }
            }

            // Return total distance
            return totalDistance;
        }

        /*
         * 
         * Performance a set of local search techniques
         * 
         */
        private void localSearch(ClusterSolution solution)
        {
            
        }

    }
}
