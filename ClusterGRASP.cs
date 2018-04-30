using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

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
    class ClusterGRASP
    {

        // Public variables
        public CluVRPInstance instance { get; set; }
        public CluVRPSolution solution { get; set; }
        
        /* 
         * 
         * Constructor
         * 
         */
        public ClusterGRASP(CluVRPInstance cluVRPinstance)
        {
            // Set instance 
            instance = cluVRPinstance;
            solution = new CluVRPSolution();            
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
        public CluVRPSolution Grasp(int totalIterations = 100, double alphaCapacity = 0.8, double alphaDistance = 1)
        {
            // Set iterator 
            int iterator = 0;
            
            // Main Cycle
            while(iterator < totalIterations)
            {
                // Construct a Greedy Randomized Solution for alpha parameter
                CluVRPSolution newSolution = constructGreedyRandomizedSolution(alphaCapacity, alphaDistance);

                // Local search 
                localSearch(newSolution);

               // Update Best solution
               if(newSolution.totalClusterRouteDistance < solution.totalClusterRouteDistance)
               {
                   solution.setClusterSolution(
                       newSolution.clusterRouteForVehicule,
                       newSolution.vehicleRemSpace, 
                       newSolution.totalClusterRouteDistance
                       );
               }

               // Increace iterator
               iterator++;
            }

            return this.solution;
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
        private CluVRPSolution constructGreedyRandomizedSolution(double alphaDemand, double alphaDistance)
        {
            // Init variables
            int numberOfVehicles = instance.vehicles;
            int vehicleCapacity = instance.capacity;
            int[][] clusters = instance.clusters;
            int numbersOfClusters = clusters.Length;
            int[] clustersDemand = instance.clusters_demand;
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
                int clusterSelected = Functions.selectRandomElement(clusterByDemandRCL);

                // Create RCL for vehicle of clusterSeleted by distance (and bestFit capacity)
                List<int> vehicleBydistanceRCL = buildVehicleByDistanceRCL(clusterRouteForVehicle, vehicleCapacity, vehicleRemSpace, clusterSelected, clustersDemand[clusterSelected], alphaDistance);

                // Only add the cluster to the route of vehicle if were possible fit it
                if (vehicleBydistanceRCL.Count > 0)
                {
                    // Select vehicle from RCL 
                    int vehicleSelected = Functions.selectRandomElement(vehicleBydistanceRCL);

                    // Add cluster to vehicle route
                    clusterRouteForVehicle[vehicleSelected].Add(clusterSelected);

                    // Update vehicle remmaing space
                    vehicleRemSpace[vehicleSelected] -= clustersDemand[clusterSelected];

                    // Remove cluster of the list to visit
                    clustersToVisit.Remove(clusterSelected);
                }
            }

            // If there are clusters without vehicle throw exception
            if(clustersToVisit.Count != 0)
            {
                throw new Exception("Greedy at Cluster-Level Error - ClusterToVisit list is not empty!");
            }

            // Add depot as final cluster for all travels
            for(int i = 0; i < numberOfVehicles; i++)
            {
                clusterRouteForVehicle[i].Add(0);
            }

            // Calculte total inter-cluster distance
            double travelTotalDistance = calculateClusterTravelDistance(clusterRouteForVehicle, instance.clustersDistanceMatrix);

            // Set solution
            CluVRPSolution newSolution = new CluVRPSolution();
            newSolution.setClusterSolution(clusterRouteForVehicle, vehicleRemSpace, travelTotalDistance);

            // Return solution
            return newSolution;
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
        private List<int> buildVehicleByDistanceRCLBestFit(IList[] clusterRouteForVehicle, int vehicleCapacity, int[] vechiculeRemSpace, int clusterSelected, int clusterDemand, double alphaDistance)
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
                double distanceBetweenClusters = instance.clustersDistanceMatrix[lastClusterVisited][clusterSelected];

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
                int lastClusterVisited = (int)clusterRouteForVehicle[j][clusterRouteForVehicle[j].Count - 1];
                double distanceBetweenClusters = instance.clustersDistanceMatrix[lastClusterVisited][clusterSelected];

                // If there is space on vehicle j AND
                // The remaning space is less than minCapacity AND
                // the distance es acceptable for RCL condition
                if (vechiculeRemSpace[j] >= clusterDemand && distanceBetweenClusters <= RCLCondition)
                {
                    RCL.Add(j);
                }

            }

            // If RCL is empty insert the vehicle that fit
            if (RCL.Count == 0)
            {
                for(int j = 0; j< numberOfVehicles; j++)
                {
                    if(vechiculeRemSpace[j] >= clusterDemand)
                    {
                        RCL.Add(j);
                    }
                }

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
                ret = Math.Max(ret, instance.clustersDistanceMatrix[lastCluster][toCluster]);
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
                ret = Math.Min(ret, instance.clustersDistanceMatrix[lastCluster][toCluster]);
            }
            return ret;
        }
       
        /*
         *
         * Calculate the total distance of the cluster travel
         *
         */ 
        public static double calculateClusterTravelDistance(List<int>[] travel, double[][] clustersDistanceMatrix)
        {
            // Set variables
            double totalDistance = 0;

            // Iterate each vehicle
            for (int vehicleNumber = 0; vehicleNumber < travel.Length; vehicleNumber++)
            {
                totalDistance += calculateClusterTravelDistance(travel[vehicleNumber], clustersDistanceMatrix);
            }

            // Return total distance
            return totalDistance;
        }

        /*
        *
        * Calculate the total distance of the cluster travel
        *
        */
        public static double calculateClusterTravelDistance(List<int> travel, double[][] clustersDistanceMatrix)
        {
            // Set variables
            double totalDistance = 0;

            // Iterate each cluster on vehicle route
            for (int clusterIt = 0; clusterIt + 1 < travel.Count; clusterIt++)
            {
                int fromCluster = travel[clusterIt];
                int ToCluster = travel[clusterIt + 1];
                totalDistance += clustersDistanceMatrix[fromCluster][ToCluster];
            }

            // Return total distance
            return totalDistance;
        }

        public static double estimateClusterTravelDistance(List<int>[] travel, double[][] clustersDistanceMatrix, double[] suffleAverageClusterDistance)
        {
            // Init distance
            double totalDistance = 0;

            // Iterate each cluster on vehicle route
            for (int vehicle = 0; vehicle < travel.Length; vehicle++)
            {
                totalDistance += estimateClusterTravelDistance(travel[vehicle], clustersDistanceMatrix, suffleAverageClusterDistance);
            }

            // Return total distance
            return totalDistance;
        }
        
        public static double estimateClusterTravelDistance(List<int> travel, double[][] clustersDistanceMatrix, double[] suffleAverageClusterDistance)
        {
            // Set variables
            double totalDistance = 0;

            // Iterate each cluster on vehicle route
            for (int clusterIt = 0; clusterIt + 1 < travel.Count; clusterIt++)
            {
                int fromCluster = travel[clusterIt];
                int ToCluster = travel[clusterIt + 1];
                totalDistance += clustersDistanceMatrix[fromCluster][ToCluster];
                totalDistance += suffleAverageClusterDistance[fromCluster];
            }

            // Return total distance
            return totalDistance;
        }

        /*
         * 
         * Performance a set of local search techniques
         * 
         */
        private void localSearch(CluVRPSolution newSolution)
        {
            // Create a local search handler for cluster-level problem
            ClusterLocalSearch localSearchsCluster = new ClusterLocalSearch(newSolution, instance, 200, 200);

            // Perform interVehicle Swap
            localSearchsCluster.swapVehicle(instance.clusters_demand);
            localSearchsCluster.interVehicleRandomSwap();
            
            localSearchsCluster.insertVehicle(instance.clusters_demand);


            // Perform interVehicle Insert
            localSearchsCluster.interVehicleRandomInsert(instance.clusters_demand);

            // Perform TwoOpt
            localSearchsCluster.twoOpt();

            // Perform Relocate
            localSearchsCluster.relocate();

            // Perform Exchange
            localSearchsCluster.exchange();                     
        }

    }
}
