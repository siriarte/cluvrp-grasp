using System;
using System.Collections.Generic;
using System.Linq;

namespace cluvrp_grasp
{
    class CustomerGRASP
    {
        // Public variables
        public CluVRPInstance instance;
        public CluVRPSolution solution;

        /*
         * 
         * Constructor 
         *
         */
        public CustomerGRASP(CluVRPInstance instance, CluVRPSolution solution)
        {
            // Set variables
            this.instance = instance;
            this.solution = solution;
        }

        /*
          * 
          * Grasp():
          * M <- calculateCustomerDistance
          * BestSolution = 0
          * While(StopCondition)
          *      Solution <- ConstructGreedySolution()
          *      NewSolution <- LocalSearch(solution)
          *      if NewSolution isBetterThan BestSolution
          *          BestSolution = NewSolution
          * return BestSolution
          *
          */
        public void Grasp(int totalIterations = 100, double alpha = 0)
        {
            // Set iterator
            int iterator = 0;

            // Main cycle
            while (iterator < totalIterations)
            {
                // Calculate new initial solution
                CluVRPSolution newSolution = constructGreedyRandomizedSolution(alpha);

                // Local search 
                this.localSearch(newSolution);

                // Update Best solution
                if (newSolution.totalCustomerRouteDistance < solution.totalCustomerRouteDistance)
                {
                    solution.setCostumerSolution(newSolution.customersPaths, newSolution.vehiculeRouteDistance);
                }

                // Increace iterator
                iterator++;
            }
            
            //End
            return;
        }

        /*
         * 
         * Applies a list of local searchs  
         *
         */ 
        private void localSearch(CluVRPSolution solution)
        {
            // Create a local search handler for cluster-level problem
            CustomerLocalSearch customerLocalSearch = new CustomerLocalSearch(solution, instance, 100, 100, 100);

            // Perform TwoOpt
            customerLocalSearch.twoOpt();

            // Perform Relocate
            customerLocalSearch.relocate();

            // Perform Exchange
            //customerLocalSearch.exchange();

            // Set the solution
            solution = customerLocalSearch.solution;
        }

        /*
         * 
         *  Create a complete Greedy Randomized Solution (with cluster and customers)
         * 
         */
        private CluVRPSolution constructGreedyRandomizedSolution(double alpha)
        {
            // Init variables
            int[][] originalClusters = instance.clusters;
            List<int>[] clusterRoute = solution.clusterRouteForVehicule;
            List<int>[][] customersCircuit = new List<int>[clusterRoute.Length][];
            Tuple<int, int> customersConnectClusters = new Tuple<int, int>(0, 0);
            double[] vehiculeTotalDistance = new double[instance.vehicles];

            // Start from depot
            int startingCustomer = 1;

            int[][][] clusterRouteWithCustomers = this.clusterRouteWithCustomers(originalClusters, clusterRoute);

            // For each vehicule cluster-route
            for (int vehicle = 0; vehicle < clusterRoute.Length; vehicle++)
            {

                // For each cluster in the i-vehicle route
                int numbersOfClusters = clusterRoute[vehicle].Count;
                int[][] clustersOneVehicle = clusterRouteWithCustomers[vehicle];
                customersCircuit[vehicle] = new List<int>[numbersOfClusters];

                // Visit all cluster for the i-vehicle
                for (int i = 0; i < numbersOfClusters; i++)
                {
                    // Add actual (initial of cluster) customer
                    customersCircuit[vehicle][i] = new List<int>();
                    customersCircuit[vehicle][i].Add(startingCustomer);

                    // For best customer to conect actual cluster to the next
                    // If the cluster only has 1 customer, you only have to know the 
                    // customer for the next cluster
                    if (i + 1 < numbersOfClusters && clustersOneVehicle[i].Length > 1)
                    {
                        customersConnectClusters = bestCustomersBetween2Clusters(clustersOneVehicle[i], clustersOneVehicle[i + 1], startingCustomer);
                    }
                    else if (i + 1 < numbersOfClusters && clustersOneVehicle[i].Length == 1)
                    {
                        int nextCustomer = bestNextCustomer(startingCustomer, clustersOneVehicle[i + 1]);
                        customersConnectClusters = new Tuple<int, int>(startingCustomer, nextCustomer);
                    }

                    // Convert array to list
                    List<int> customersToVisit = clustersOneVehicle[i].OfType<int>().ToList();

                    // Remove initial and final customers
                    customersToVisit.Remove(startingCustomer);
                    customersToVisit.Remove(customersConnectClusters.Item1);

                    // While exists customers to visit
                    while (customersToVisit.Count > 0)
                    {
                        // Create RCL for customer 
                        List<int> customerRCL = buildCustomerRCL(startingCustomer, customersToVisit, alpha);

                        // Select customer for RCL
                        int customerSelected = Functions.selectRandomElement(customerRCL);

                        // Add customer to the path
                        customersCircuit[vehicle][i].Add(customerSelected);

                        // Quit visited customer
                        customersToVisit.Remove(customerSelected);
                    }

                    // Add final customer that connect to i+1 cluster
                    // In the final cluster the next customer is 0 
                    // the it has not be added
                    if (clustersOneVehicle[i].Length > 1 && i + 1 < numbersOfClusters)
                    {
                        customersCircuit[vehicle][i].Add(customersConnectClusters.Item1);
                    }

                    // Next customer of next cluster 
                    startingCustomer = customersConnectClusters.Item2;
                }

                // Calculte total inter-cluster distance
                vehiculeTotalDistance[vehicle] = Functions.calculateTotalTravelDistance(customersCircuit, instance.customersDistanceMatrix, vehicle);
            }

            // Set solution
            CluVRPSolution newSolution = new CluVRPSolution();
            newSolution.setCostumerSolution(customersCircuit, vehiculeTotalDistance);

            // Return solution
            return newSolution;
        }

        /*
         * 
         * Add the list of customers to a cluster route for vehicle
         * 
         */
        private int[][][] clusterRouteWithCustomers(int[][] notSortedclusters, List<int>[] clusterRoute)
        {
            // Create returned array
            int[][][] sortedClusterRoute = new int[clusterRoute.Length][][];
            
            // For each vehicle
            for(int vehicle = 0; vehicle < clusterRoute.Length; vehicle++)
            {
                sortedClusterRoute[vehicle] = new int[clusterRoute[vehicle].Count][];

                // For each cluster of vehicle
                for (int i = 0; i < clusterRoute[vehicle].Count; i++)
                {
                    // Add to the list of costumers to the cluster i
                    int actualCluster = clusterRoute[vehicle][i];
                    sortedClusterRoute[vehicle][i] = notSortedclusters[actualCluster];
                }
            }

            // Return value
            return sortedClusterRoute;
        }

        /*
         *
         * Select the best next customer of 'cluster' from 'startingNode' 
         * Using min distance as criteria
         * 
         */
        private int bestNextCustomer(int startingNode, int[] cluster)
        {
            // Define min distance and tuple
            double minDistance = double.MaxValue;
            int ret = 0;

            // For each customer in cluster 1
            for (int i = 0; i < cluster.Length; i++)
            {
                // Customer 1
                int customer1 = cluster[i];

                // Get distance between customer 1 and customer 2
                double distance = instance.customersDistanceMatrix[startingNode][customer1];

                // Update min distance
                if (distance < minDistance && customer1 != startingNode)
                {
                    minDistance = distance;
                    ret = customer1;
                }
            }

            // Return min distance
            return ret;
        }

        /*
         * 
         * Create RCL list with min distance betweeen customers as criteria 
         * 
         */
        private List<int> buildCustomerRCL(int actualCustomer, List<int> customersToVisit, double alpha)
        {
            // Set variables
            List<int> RCL = new List<int>();

            // Calculate max and min distance for RCL condition
            double minDistance = Functions.minCustomerDistance(customersToVisit, actualCustomer, instance.customersDistanceMatrix);
            double maxDistance = Functions.maxCustomerDistance(customersToVisit, actualCustomer, instance.customersDistanceMatrix);

            // Set RCL condition criteria
            double RCLCondition = minDistance + alpha * (maxDistance - minDistance);

            // For each vehicle
            for (int j = 0; j < customersToVisit.Count; j++)
            {
                // Calculate customers distance
                double distanceBetweenCustomers = instance.customersDistanceMatrix[actualCustomer][customersToVisit[j]];

                // Add the vehicle to RCL if is possible
                if (distanceBetweenCustomers <= RCLCondition)
                {
                    RCL.Add(customersToVisit[j]);
                }
            }

            // return rcl
            return RCL;
        }

        /*
         * 
         * Return the two customers that has min distance bewteen two clusters
         * Avoiding using the 'startingNode' as the sources
         * 
         */
        private Tuple<int, int> bestCustomersBetween2Clusters(int[] cluster1, int[] cluster2, int startingNode)
        {

            // Define min distance and tuple
            double minDistance = double.MaxValue;
            Tuple<int, int> ret = new Tuple<int, int>(0, 0);

            // For each customer in cluster 1
            for (int i = 0; i < cluster1.Length; i++)
            {
                // Customer 1
                int customer1 = cluster1[i];

                // For each customer in cluster 2
                for (int j = 0; j < cluster2.Length; j++)
                {
                    // Customer 2
                    int customer2 = cluster2[j];

                    // Get distance between customer 1 and customer 2
                    double distance = instance.customersDistanceMatrix[customer1][customer2];

                    // Update min distance
                    if (distance < minDistance && customer1 != startingNode)
                    {
                        minDistance = distance;
                        ret = new Tuple<int, int>(cluster1[i], cluster2[j]);
                    }
                }
            }

            // Return min distance
            return ret;
        }

    }
}
