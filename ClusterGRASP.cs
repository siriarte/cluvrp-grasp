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
    enum FitAlgorithm {RCL, BestFit, FirstFit};
    class ClusterGRASP
    {

        // Public variables
        public CluVRPInstance instance { get; set; }
        public CluVRPSolution solution { get; set; }
        public FitAlgorithm fitAlgorithm { get; set; }
        public int[] fitAlgorithmCounter { get; set; }

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
            fitAlgorithm = FitAlgorithm.RCL;
            fitAlgorithmCounter = new int[3];        
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
                try
                {
                    CluVRPSolution newSolution = constructGreedyRandomizedSolution(alphaCapacity, alphaDistance);
                    
                    // Local search 
                    localSearch(newSolution);

                    // Update Best solution
                    if (newSolution.totalClusterRouteDistance < solution.totalClusterRouteDistance)
                    {
                        solution.setClusterSolution(
                            newSolution.clusterRouteForVehicule,
                            newSolution.vehicleRemSpace,
                            newSolution.totalClusterRouteDistance
                            );
                        solution.fitUsed = fitAlgorithm;
                    }

                    // Count the fit algorithm success solution
                    fitAlgorithmCounter[(int)fitAlgorithm]++;
                }
                catch(Exception)
                {
                    // do nothing, it is handled on the constructGreedyRandomizedSolution
                }

                // Increace iterator
                iterator++;
            }

            // Set the counter of fit algorithm on solution for reports
            solution.fitAlgorithmCounter = fitAlgorithmCounter;

            // Return best solution
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

            // Bluid a cluster route for a vehicle depending on selected fit algorithm
            if (fitAlgorithm == FitAlgorithm.RCL)
            {
                buildClusterRouteForVehicleByRCL(clusterRouteForVehicle, clustersToVisit, clustersDemand, alphaDemand, alphaDistance, vehicleCapacity, vehicleRemSpace);
            }
            else if (fitAlgorithm == FitAlgorithm.BestFit)
            {
                buildClusterRouteForVehicleByBestFit(clusterRouteForVehicle, clustersToVisit, clustersDemand, alphaDemand, alphaDistance, vehicleCapacity, vehicleRemSpace);

                // Always return to RCL algorithm
                fitAlgorithm = FitAlgorithm.RCL;
            }

            // If there are clusters without vehicle try swaping clusters to win space and inserts the remaining clusters
            if (clustersToVisit.Count  != 0)
            {
                forceSwapFitCluster(clusterRouteForVehicle, vehicleRemSpace, clustersToVisit);
            }

            // If still are clusters without vehicle throw exception and change fit algorithm
            if (clustersToVisit.Count != 0)
            {
                if (fitAlgorithm == FitAlgorithm.RCL)
                {
                    fitAlgorithm = FitAlgorithm.BestFit;
                }
                throw new Exception("Greedy at Cluster-Level Error - ClusterToVisit list is not empty on " + fitAlgorithm + " algorithm");
            }
      
            // Add depot as final cluster for all travels
            for (int i = 0; i < numberOfVehicles; i++)
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
            
        private void buildClusterRouteForVehicleByBestFit(List<int>[] clusterRouteForVehicle, List<int> clustersToVisit, int[] clustersDemand, double alphaDemand, double alphaDistance, int vehicleCapacity, int[] vehicleRemSpace)
        {
            int res = 0;
            int numberOfClusters = clustersToVisit.Count;
            List<int> clusterToIterate = new List<int>(clustersToVisit);
            Functions.sortClusterByDemand(clusterToIterate, instance.clusters_demand);

            for (int i = 0; i < clusterToIterate.Count; i++)
            {
                int cluster = clusterToIterate[i];
                int clusterDemand = instance.clusters_demand[cluster];
                int j;
                int minCapacity = instance.capacity + 1;
                int bestIndex = 0;

                // For each vehicle
                for (j = 0; j <= res; j++)
                {
                    // If there is space on vehicle j AND
                    // The remaning space is less than minCapacity AND
                    // the distance es acceptable for RCL condition

                    if (vehicleRemSpace[j] >= clusterDemand &&
                        vehicleRemSpace[j] - clusterDemand < minCapacity)
                    {
                        bestIndex = j;
                        minCapacity = vehicleRemSpace[j] - clusterDemand;
                    }

                }
                if (minCapacity == instance.capacity + 1)
                {
                    if (res + 1 < vehicleRemSpace.Length)
                    {
                        res++;
                        vehicleRemSpace[res] = instance.capacity - clusterDemand;
                        clusterRouteForVehicle[res].Add(cluster);
                        clustersToVisit.Remove(cluster);
                    }
                }
                else
                {
                    vehicleRemSpace[bestIndex] -= clusterDemand;
                    clusterRouteForVehicle[bestIndex].Add(cluster);
                    clustersToVisit.Remove(cluster);
                }
            }

        }

        private void buildClusterRouteForVehicleByRCL(List<int>[] clusterRouteForVehicle, List<int> clustersToVisit, int[] clustersDemand, double alphaDemand, double alphaDistance, int vehicleCapacity, int[] vehicleRemSpace)
        {
            int numbersOfClusters = clustersToVisit.Count;
            for (int i = 0; i + 1 < numbersOfClusters; i++)
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
        }

        private void forceInsertFitCluster(List<int>[] clusterRouteForVehicle, int[] vehicleRemSpace, List<int> clustersToVisit)
        {
            foreach (int cluster in clustersToVisit)
            {
                int spaceNeed = instance.clusters_demand[cluster];

                for(int vehicle = 0; vehicle < clusterRouteForVehicle.Length; vehicle++)
                {

                }
            }

        }

        private void forceSwapFitCluster(List<int>[] clusterRouteForVehicle, int[] vehicleRemSpace, List<int> clustersToVisit)
        {
            int numberOfClusterToFit = clustersToVisit.Count;
            for (int clusterIt = 0; clusterIt< numberOfClusterToFit; clusterIt++) {

                int clusterToInsert = clustersToVisit[clusterIt];
                int neededSpace = instance.clusters_demand[clusterToInsert];
                bool wasFit = false;

                for (int vehicle1 = 0; vehicle1 < clusterRouteForVehicle.Length; vehicle1++)
                {
                    if (wasFit) break;
                    for (int vehicle2 = 0; vehicle2 < clusterRouteForVehicle.Length; vehicle2++)
                    {
                        if (wasFit) break;
                        if (vehicle1 != vehicle2)
                        {
                            for (int clusterItV1 = 1; clusterItV1 < clusterRouteForVehicle[vehicle1].Count; clusterItV1++)
                            {
                                if (wasFit) break;
                                int clusterV1 = clusterRouteForVehicle[vehicle1][clusterItV1];
                                int clusterV1Demand = instance.clusters_demand[clusterV1];

                                for (int clusterItV2 = 0; clusterItV2 < clusterRouteForVehicle[vehicle2].Count; clusterItV2++)
                                {
                                    if (clusterItV1 != 0 && clusterItV2 != 0)
                                    {
                                        int clusterV2 = clusterRouteForVehicle[vehicle2][clusterItV2];
                                        int clusterV2Demand = instance.clusters_demand[clusterV2];

                                        int remSpaceV1 = vehicleRemSpace[vehicle1] + clusterV1Demand - clusterV2Demand;
                                        int remSpaceV2 = vehicleRemSpace[vehicle2] + clusterV2Demand - clusterV1Demand;
                                        
                                        if ((remSpaceV1 >= neededSpace || remSpaceV2 >= neededSpace) && remSpaceV1 >= 0 && remSpaceV2 >= 0)
                                        {
                                            clusterRouteForVehicle[vehicle1][clusterItV1] = clusterV2;
                                            clusterRouteForVehicle[vehicle2][clusterItV2] = clusterV1;
                                            vehicleRemSpace[vehicle1] += clusterV1Demand - clusterV2Demand;
                                            vehicleRemSpace[vehicle2] += clusterV2Demand - clusterV1Demand;
                                            clustersToVisit.Remove(clusterToInsert);
                                            wasFit = true;

                                            if (remSpaceV1 >= neededSpace)
                                            {

                                                clusterRouteForVehicle[vehicle1].Insert(clusterItV1, clusterToInsert);
                                                vehicleRemSpace[vehicle1] -= neededSpace;
                                            }
                                            else if (remSpaceV2 >= neededSpace)
                                            {
                                                clusterRouteForVehicle[vehicle2].Insert(clusterItV2, clusterToInsert);
                                                vehicleRemSpace[vehicle2] -= neededSpace;
                                            }
                                        }
                                        if (wasFit) break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
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
            ClusterLocalSearch localSearchsCluster = new ClusterLocalSearch(newSolution, instance, 100, 100, 100);

            // Perform interVehicle Swap
            localSearchsCluster.swapVehicle(instance.clusters_demand);

            // Perform interVehicle Insert
            localSearchsCluster.insertVehicle(instance.clusters_demand);

            // Random versions
            /*
            localSearchsCluster.interVehicleRandomInsert(instance.clusters_demand);
            localSearchsCluster.interVehicleRandomSwap();
            */

            // Perform TwoOpt
            localSearchsCluster.twoOpt();

            // Perform Relocate
            localSearchsCluster.relocate();

            // Perform Exchange
            localSearchsCluster.exchange();   
        }

    }
}
