using System;
using System.Collections;
using System.Collections.Generic;

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
    enum FitAlgorithm {RCL, BestFit, RCLError, BestFitError };
    enum LocalSearch { InsertVehicle, SwapVehicle, RndInsertVehicle, RndSwapVehicle,
                       TwoOpt, Relocate, Exchange };

    class ClusterGRASP
    {
        // Public variables
        public CluVRPInstance instance { get; set; }
        public CluVRPSolution solution { get; set; }
        public int[] fitAlgorithmCounter { get; set; }
        private Parameters parameters { get; set; }
        private FitAlgorithm fitAlgorithm { get; set; }
        private bool bestFitFirstExecution { get; set; }
        private bool selectFitAlgorithmRandom { get; set; }
        private Random rndFitAlgorithm;
        private List<LocalSearch> localSearchsOrder;

        /* 
         * 
         * Constructor
         * 
         */
        public ClusterGRASP(CluVRPInstance cluVRPinstance, Parameters parameters)
        {
            // Set instance 
            instance = cluVRPinstance;
            solution = new CluVRPSolution();
            fitAlgorithm = parameters.Cluster_FitAlgoritm;
            bestFitFirstExecution = true;
            selectFitAlgorithmRandom = false;
            rndFitAlgorithm = new Random();
            fitAlgorithmCounter = new int[4];
            this.parameters = parameters;

            // For local search execute order
            if (parameters.Cluster_LS_Order.Length == 0)
            {
                this.localSearchsOrder = new List<LocalSearch> { LocalSearch.InsertVehicle, LocalSearch.SwapVehicle,
                LocalSearch.RndInsertVehicle, LocalSearch.RndSwapVehicle, LocalSearch.TwoOpt,
                LocalSearch.Relocate, LocalSearch.Exchange};
            }else
            {
                this.localSearchsOrder = new List<LocalSearch>();
                for (int i = 0; i < parameters.Cluster_LS_Order.Length; i++)
                {
                    localSearchsOrder.Add((LocalSearch)parameters.Cluster_LS_Order[i]);
                }
            }
            
            // End of Contructor
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
        public CluVRPSolution Grasp()
        {
            // Set iterator 
            int iterator = 0;
            int totalIterations = parameters.Cluster_GRASPIterations;
            double alphaCapacity = parameters.Cluster_AlphaCapacity;
            double alphaDistance = parameters.Cluster_AlphaDistance;

            // Main Cycle
            while (iterator < totalIterations)
            {
                // Construct a Greedy Randomized Solution for alpha parameter
                try
                {
                    // Create a Greedy Randomized Solution
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
                        solution.bestClusterLSOrder = localSearchsOrder;
                    }

                    // Count the fit algorithm success solution
                    fitAlgorithmCounter[(int)fitAlgorithm]++;


                }
                catch(Exception)
                {
                    // do nothing, it is handled on the constructGreedyRandomizedSolution
                    // Count error on fit algorithm
                    if (fitAlgorithm == FitAlgorithm.RCL)
                        fitAlgorithmCounter[(int)FitAlgorithm.RCLError]++;
                    else if (fitAlgorithm == FitAlgorithm.BestFit)
                        fitAlgorithmCounter[(int)FitAlgorithm.BestFitError]++;
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

            // If random critearia is activated for select fit algorithm
            if (selectFitAlgorithmRandom)
            {
                fitAlgorithm = (FitAlgorithm) rndFitAlgorithm.Next(0, 2);
            }

            // Bluid a cluster route for a vehicle depending on selected fit algorithm
            if (fitAlgorithm == FitAlgorithm.RCL)
            {
                fitAlgorithmCounter[(int)FitAlgorithm.RCL]++;
                buildClusterRouteForVehicleByRCL(clusterRouteForVehicle, clustersToVisit, clustersDemand, alphaDemand, alphaDistance, vehicleCapacity, vehicleRemSpace);
            }
            else if (fitAlgorithm == FitAlgorithm.BestFit)
            {
                fitAlgorithmCounter[(int)FitAlgorithm.BestFit]++;
                buildClusterRouteForVehicleByBestFit(clusterRouteForVehicle, clustersToVisit, clustersDemand, alphaDemand, alphaDistance, vehicleCapacity, vehicleRemSpace);
            }

            // If there are clusters without vehicle try swaping clusters to win space and inserts the remaining clusters
            if (clustersToVisit.Count  != 0)
            {
                forceSwapFitCluster(clusterRouteForVehicle, vehicleRemSpace, clustersToVisit);
            }

            // If still are clusters without vehicle throw exception and change fit algorithm
            if (clustersToVisit.Count != 0)
            {
                // Start with random variations on fit algorithm selection
                selectFitAlgorithmRandom = true;

                // Throw exception
                throw new Exception("Greedy at Cluster-Level Error - ClusterToVisit list is not empty on " + fitAlgorithm + " algorithm");
            }
      
            // Add depot as final cluster for all travels
            for (int i = 0; i < numberOfVehicles; i++)
            {
                clusterRouteForVehicle[i].Add(0);
            }

             // Calculte total inter-cluster distance
            double travelTotalDistance = Functions.calculateTotalClusterTravelDistance(clusterRouteForVehicle, instance.clustersDistanceMatrix);

            // Set solution
            CluVRPSolution newSolution = new CluVRPSolution();
            newSolution.setClusterSolution(clusterRouteForVehicle, vehicleRemSpace, travelTotalDistance);

            // Return solution
            return newSolution;
        }

        /*
         * 
         *  Build a cluster route using BestFit algorithm 
         * 
         */
        private void buildClusterRouteForVehicleByBestFit(List<int>[] clusterRouteForVehicle, List<int> clustersToVisit, int[] clustersDemand, double alphaDemand, double alphaDistance, int vehicleCapacity, int[] vehicleRemSpace)
        {
            // Init variables
            int usedVehicles = 0;
            int numberOfClusters = clustersToVisit.Count;
            List<int> clusterToIterate = new List<int>(clustersToVisit);

            // If random option is activated
            if (!bestFitFirstExecution)
            {
                // Generate the order of the select clusters random
                Functions.Shuffle(new Random(), clusterToIterate);  
            }
            else
            {
                // Generate the order of the select clusters sorted by min capacity
                Functions.sortClusterByDemand(clusterToIterate, instance.clusters_demand);

                // Set first execution to false
                bestFitFirstExecution = false;
            }

            // For each intermediate cluster (not init and end)
            for (int i = 0; i < clusterToIterate.Count; i++)
            {
                // Set cluster and its demanad
                int cluster = clusterToIterate[i];
                int clusterDemand = instance.clusters_demand[cluster];

                // Init min capacity on max value
                int minCapacity = instance.capacity + 1;
                
                // Init bestIndex
                int bestIndex = 0;

                // For each used vehicle
                for (int j = 0; j <= usedVehicles; j++)
                {
                    // If there is space on vehicle j AND
                    // The remaning space is less than minCapacity AND
                    // the distance es acceptable for RCL condition
                    if (vehicleRemSpace[j] >= clusterDemand &&
                        vehicleRemSpace[j] - clusterDemand < minCapacity)
                    {
                        // Set new best index and update new min capacity
                        bestIndex = j;
                        minCapacity = vehicleRemSpace[j] - clusterDemand;
                    }

                }

                // If there is not capacity available
                // use new vehicle is neccesary
                if (minCapacity == instance.capacity + 1)
                {
                    // If there is one more vehicle available
                    if (usedVehicles + 1 < vehicleRemSpace.Length)
                    {
                        // Increase used vehicles
                        usedVehicles++;

                        // Put the cluster on the new vehicle
                        vehicleRemSpace[usedVehicles] = instance.capacity - clusterDemand;
                        clusterRouteForVehicle[usedVehicles].Add(cluster);

                        // Remove cluster on the list
                        clustersToVisit.Remove(cluster);
                    }
                }
                // If there is capacity on the used vehicles
                else
                {
                    // Put the cluster on the best vehicle 
                    vehicleRemSpace[bestIndex] -= clusterDemand;
                    clusterRouteForVehicle[bestIndex].Add(cluster);

                    // Remove cluster on the list
                    clustersToVisit.Remove(cluster);
                }
            } // End for cluster

            // End
            return;
        }

        /*
         * 
         *  Build a cluster route using RCL criteria 
         * 
         */
        private void buildClusterRouteForVehicleByRCL(List<int>[] clusterRouteForVehicle, List<int> clustersToVisit, int[] clustersDemand, double alphaDemand, double alphaDistance, int vehicleCapacity, int[] vehicleRemSpace)
        {
            // Get number of clusters to visit
            int numbersOfClusters = clustersToVisit.Count;

            // For each cluster
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

        /*
         * 
         * Try to insert the cluster doesn't have vehicle swapping
         * clusters on differents vehicles to generate the
         * necessary space to fit it.
         * 
         */
        private void forceSwapFitCluster(List<int>[] clusterRouteForVehicle, int[] vehicleRemSpace, List<int> clustersToVisit)
        {
            // Get number of cluster does not have vehicle
            int numberOfClusterToFit = clustersToVisit.Count;

            // For each cluster doens't have vehicle
            for (int clusterIt = 0; clusterIt< numberOfClusterToFit; clusterIt++) {

                // Get cluster to insert and its demand
                int clusterToInsert = clustersToVisit[clusterIt];
                int neededSpace = instance.clusters_demand[clusterToInsert];

                // No fit yet
                bool wasFit = false;

                // For each vehicle 2
                for (int vehicle1 = 0; vehicle1 < clusterRouteForVehicle.Length; vehicle1++)
                {
                    // If fit break
                    if (wasFit) break;

                    // For each vehicle 2
                    for (int vehicle2 = 0; vehicle2 < clusterRouteForVehicle.Length; vehicle2++)
                    {
                        // If fit break
                        if (wasFit) break;

                        // For every distinct vehicles
                        if (vehicle1 != vehicle2)
                        {
                            // For each cluster 1 on vehicle 1
                            for (int clusterItV1 = 1; clusterItV1 < clusterRouteForVehicle[vehicle1].Count; clusterItV1++)
                            {
                                // If fit break
                                if (wasFit) break;

                                // Get cluster 1 and its demand
                                int clusterV1 = clusterRouteForVehicle[vehicle1][clusterItV1];
                                int clusterV1Demand = instance.clusters_demand[clusterV1];

                                // For each cluster 2 on vehicle 2
                                for (int clusterItV2 = 0; clusterItV2 < clusterRouteForVehicle[vehicle2].Count; clusterItV2++)
                                {
                                    // For each cluster that no are the depot
                                    if (clusterItV1 != 0 && clusterItV2 != 0)
                                    {
                                        // Get cluster 2 and its demand
                                        int clusterV2 = clusterRouteForVehicle[vehicle2][clusterItV2];
                                        int clusterV2Demand = instance.clusters_demand[clusterV2];
                                        
                                        // Calculate the free space generated if the swap is performed
                                        int remSpaceV1 = vehicleRemSpace[vehicle1] + clusterV1Demand - clusterV2Demand;
                                        int remSpaceV2 = vehicleRemSpace[vehicle2] + clusterV2Demand - clusterV1Demand;
                                        
                                        // If swap is possible and it generate the space needed
                                        if ((remSpaceV1 >= neededSpace || remSpaceV2 >= neededSpace) && remSpaceV1 >= 0 && remSpaceV2 >= 0)
                                        {
                                            // Make the swap
                                            clusterRouteForVehicle[vehicle1][clusterItV1] = clusterV2;
                                            clusterRouteForVehicle[vehicle2][clusterItV2] = clusterV1;

                                            // Update the new free space on vehicles
                                            vehicleRemSpace[vehicle1] += clusterV1Demand - clusterV2Demand;
                                            vehicleRemSpace[vehicle2] += clusterV2Demand - clusterV1Demand;

                                            // Remove cluster from the list
                                            clustersToVisit.Remove(clusterToInsert);

                                            // Cluster fit on vehicle
                                            wasFit = true;

                                            // If the free space was on vehicle 1
                                            if (remSpaceV1 >= neededSpace)
                                            {
                                                // Insert cluster on vehicle 1
                                                clusterRouteForVehicle[vehicle1].Insert(clusterItV1, clusterToInsert);

                                                // Update free space on vehicle 1
                                                vehicleRemSpace[vehicle1] -= neededSpace;
                                            }
                                            // If the free space was on vehicle 1
                                            else if (remSpaceV2 >= neededSpace)
                                            {
                                                // Insert cluster on vehicle 1
                                                clusterRouteForVehicle[vehicle2].Insert(clusterItV2, clusterToInsert);
                                                // Update free space on vehicle 1
                                                vehicleRemSpace[vehicle2] -= neededSpace;
                                            }
                                        }
                                        // If fit break
                                        if (wasFit) break;

                                    } // End cluster 1 and cluster 2 were not depot
                                } // End for cluster 2
                            } // End for cluster 1
                        } // End for each vehicle 1 != vehicle 2
                    } //End for vehicle 1
                } // End for vehicle 2
            } // End for clusterToInsert

            //End
            return;
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
         *  and last cluster on each vehicle PLUS Best Fit criteria    
         * 
         */
        private List<int> buildVehicleByDistanceRCLAndBestFit(List<int>[] clusterRouteForVehicle, int vehicleCapacity, int[] vechiculeRemSpace, int clusterSelected, int clusterDemand, double alphaDistance)
        {
            // Set variables
            List<int> RCL = new List<int>();
            int numberOfVehicles = vechiculeRemSpace.Length;
            int minCapacity = vehicleCapacity + 1;
            int bestIndex = 0;

            // Calculate max and min distance for RCL condition
            double minDistance = Functions.minClusterDistance(clusterRouteForVehicle, clusterSelected, instance.clustersDistanceMatrix);
            double maxDistance = Functions.maxClusterDistance(clusterRouteForVehicle, clusterSelected, instance.clustersDistanceMatrix);

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
        private List<int> buildVehicleByDistanceRCL(List<int>[] clusterRouteForVehicle, int vehicleCapacity, int[] vechiculeRemSpace, int clusterSelected, int clusterDemand, double alphaDistance)
        {
            // Set variables
            List<int> RCL = new List<int>();
            int numberOfVehicles = vechiculeRemSpace.Length;
            int minCapacity = vehicleCapacity + 1;

            // Calculate max and min distance for RCL condition
            double minDistance = Functions.minClusterDistance(clusterRouteForVehicle, clusterSelected, instance.clustersDistanceMatrix);
            double maxDistance = Functions.maxClusterDistance(clusterRouteForVehicle, clusterSelected, instance.clustersDistanceMatrix);

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
         * Performance a set of local search techniques
         * 
         */
        private void localSearch(CluVRPSolution newSolution)
        {
            // Create a local search handler for cluster-level problem
            ClusterLocalSearch localSearchsCluster = new ClusterLocalSearch(newSolution, 
                instance, 
                parameters.Cluster_LS_TwoOpt_Iterations,
                parameters.Cluster_LS_Relocate_Iterations,
                parameters.Cluster_LS_Exchange_Iterations,
                parameters.Cluster_LS_RndSwapVehicle,
                parameters.Cluster_LS_RndInsertVehicle
                );


            // If random order for local searchs is activated
            if (parameters.Cluster_LS_Order.Length == 0)
            {
                Functions.Shuffle(new Random(), this.localSearchsOrder);
            }

            // Execute local search in the correct order
            for (int i = 0; i < localSearchsOrder.Count; i++)
            {
                // Next local search 
                LocalSearch ls = (LocalSearch)localSearchsOrder[i];

                // Perform interVehicle Swap
                if (ls == LocalSearch.SwapVehicle && parameters.Cluster_LS_InsertVehicle)
                {
                    localSearchsCluster.swapVehicle(instance.clusters_demand);
                }

                // Perform interVehicle Insert
                if (ls == LocalSearch.InsertVehicle && parameters.Cluster_LS_SwapVehicle)
                {
                    localSearchsCluster.insertVehicle(instance.clusters_demand);
                }

                // Perform random interVehicle Insert
                if (ls == LocalSearch.RndInsertVehicle && parameters.Cluster_LS_RndInsertVehicle != 0)
                {
                    localSearchsCluster.interVehicleRandomChange(instance.clusters_demand);
                }

                // Perform random interVehicle Swap
                if (ls == LocalSearch.RndSwapVehicle && parameters.Cluster_LS_RndSwapVehicle != 0)
                {
                    localSearchsCluster.interVehicleRandomSwap();
                }

                // Perform Two-opt
                if (ls == LocalSearch.TwoOpt && parameters.Cluster_LS_TwoOpt_Iterations != 0)
                {
                    localSearchsCluster.twoOpt();
                }

                // Perform Relocate
                if (ls == LocalSearch.Relocate && parameters.Cluster_LS_Relocate_Iterations != 0)
                {
                    localSearchsCluster.relocate();
                }

                // Perform Exchange
                if (ls == LocalSearch.Exchange && parameters.Cluster_LS_Exchange_Iterations != 0)
                {
                    localSearchsCluster.exchange();
                }

            }
        }

    }
}
