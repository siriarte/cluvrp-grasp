using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace cluvrp_grasp
{
    class CustomerGRASP
    {
        // Public variables
        public CluVRPInstance instance;
        public double[][] customersDistanceMatrix;
        public CustomerSolution bestSolution;
        public ClusterSolution clusterSolution;

        public CustomerGRASP(CluVRPInstance instance, ClusterSolution clusterSolution)
        {
            // Set variables
            this.instance = instance;
            this.clusterSolution = clusterSolution;
            this.bestSolution = new CustomerSolution(null, double.MaxValue);
            this.customersDistanceMatrix = Functions.customersDistanceMatrix(instance);
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
        public void Grasp(int totalIterations = 100, double alpha = 0.8)
        {
            int iterator = 0;
            while (iterator < totalIterations)
            {
                CustomerSolution solution = constructGreedyRandomizedSolution(alpha);

                // Local search 
                //this.localSearch(ref solution);

                // Update Best solution
                if (solution.totalRouteDistance < bestSolution.totalRouteDistance)
                {
                    bestSolution = solution;
                }

                // Increace iterator
                iterator++;
            }

            return;
        }

        private void localSearch(ref CustomerSolution solution)
        {
            throw new NotImplementedException();
        }

        private CustomerSolution constructGreedyRandomizedSolution(double alpha)
        {
            // Init variables
            int[][] notSortedClusters = instance.clusters();
            List<int>[] clusterRoute = this.clusterSolution.clusterRouteForVehicule;
            List<int>[][] customersCircuit = new List<int>[clusterRoute.Length][];
            Tuple<int, int> customersConnectClusters = new Tuple<int, int>(0, 0); 

            // Start from depot
            int startingCustomer = 0;

            int[][][] customerByClusterOrderRoute = sortClustersByRoute(notSortedClusters, clusterRoute);

            // For each vehicule cluster-route
            for (int vehicle = 0; vehicle < clusterRoute.Length; vehicle++)
            {

                // For each cluster in the i-vehicle route
                int numbersOfClusters = clusterRoute[vehicle].Count;
                int[][] clusters = customerByClusterOrderRoute[vehicle];
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
                    if (i + 1 < numbersOfClusters && clusters[i].Length > 1)
                    {
                        customersConnectClusters = bestCustomersBetween2Clusters(clusters[i], clusters[i + 1], startingCustomer);
                    }
                    else if (i + 1 < numbersOfClusters && clusters[i].Length == 1)
                    {
                        int nextCustomer = bestNextCustomer(startingCustomer, clusters[i + 1]);
                        customersConnectClusters = new Tuple<int, int>(startingCustomer, nextCustomer);
                    }

                    // Convert array to list
                    List<int> customersToVisit = clusters[i].OfType<int>().ToList();

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
                    if (clusters[i].Length > 1 && i + 1 < numbersOfClusters)
                    {
                        customersCircuit[vehicle][i].Add(customersConnectClusters.Item1);
                    }

                    // Next customer of next cluster 
                    startingCustomer = customersConnectClusters.Item2;
                }
            }
            // Calculte total inter-cluster distance
            double travelTotalDistance = Functions.calculateTotalTravelDistance(customersCircuit, this.customersDistanceMatrix);

            // Set solution
            CustomerSolution solution = new CustomerSolution(customersCircuit, travelTotalDistance);

            // Return solution
            return solution;
        }

        private int[][][] sortClustersByRoute(int[][] notSortedclusters, List<int>[] clusterRoute)
        {
            int[][][] sortedClusterRoute = new int[clusterRoute.Length][][];
            
            for(int vehicle = 0; vehicle < clusterRoute.Length; vehicle++)
            {
                sortedClusterRoute[vehicle] = new int[clusterRoute[vehicle].Count][];
                for (int i = 0; i < clusterRoute[vehicle].Count; i++)
                {
                    int actualCluster = clusterRoute[vehicle][i];
                    sortedClusterRoute[vehicle][i] = notSortedclusters[actualCluster];
                }
            }
            return sortedClusterRoute;
        }

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
                double distance = this.customersDistanceMatrix[startingNode][customer1];

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

        private List<int> buildCustomerRCL(int actualCustomer, List<int> customersToVisit, double alpha)
        {
            // Set variables
            List<int> RCL = new List<int>();

            // Calculate max and min distance for RCL condition
            double minDistance = minCustomerDistance(customersToVisit, actualCustomer);
            double maxDistance = maxCustomerDistance(customersToVisit, actualCustomer);

            // Set RCL condition criteria
            double RCLCondition = minDistance + alpha * (maxDistance - minDistance);

            // For each vehicle
            for (int j = 0; j < customersToVisit.Count; j++)
            {

                // Calculate customers distance
                double distanceBetweenCustomers = this.customersDistanceMatrix[actualCustomer][customersToVisit[j]];

                // Add the vehicle to RCL if is possible
                if (distanceBetweenCustomers <= RCLCondition)
                {
                    RCL.Add(customersToVisit[j]);
                }
            }

            // return rcl
            return RCL;
        }

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
                    double distance = this.customersDistanceMatrix[customer1][customer2];

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

        /*
        * 
        * Calculte the max distance between the last cluster 
        * visited (of all vehicles) and toCluster
        * 
        */
        private double maxCustomerDistance(List<int> customers, int toCustomer)
        {
            double ret = double.MinValue;
            for (int i = 0; i < customers.Count; i++)
            {
               ret = Math.Max(ret, this.customersDistanceMatrix[customers[i]][toCustomer]);

            }
            return ret;
        }

        /*
         * 
         * Calculte the min distance between the last cluster 
         * visited (of all vehicles) and toCluster
         * 
         */
        private double minCustomerDistance(List<int> customers, int toCustomer)
        {
            double ret = double.MaxValue;
            for (int i = 0; i < customers.Count; i++)
            {
                ret = Math.Min(ret, this.customersDistanceMatrix[customers[i]][toCustomer]);
            }
            return ret;
        }

    }
}
