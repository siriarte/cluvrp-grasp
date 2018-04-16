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
        public double[][] clustersDistanceMatrix;
        public CustomerSolution bestSolution;
        
        public CustomerGRASP(CluVRPInstance instance)
        {
            // Set variables
            this.instance = instance;
            this.bestSolution = new CustomerSolution(null, double.MaxValue);            
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
            while(iterator < totalIterations)
            {
                CustomerSolution solution = constructGreedyRandomizedSolution(alpha);

                // Local search 
                this.localSearch(ref solution);

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
            int[][] clusters = instance.clusters();
            int numbersOfClusters = clusters.Length;

            // Main Cycle 
            // Visitir all cluster and create TSP of customers for each one
            for (int i = 0; i + 1 < numbersOfClusters; i++)
            {

                // Convert array to list
                List<int> customers = clusters[i].OfType<int>().ToList();

                while()

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
            if (clustersToVisit.Count != 0)
            {
                return new ClusterSolution(new List<int>[0], 0);
            }

            // Add depot as final cluster for all travels
            for (int i = 0; i < numberOfVehicles; i++)
            {
                clusterRouteForVehicle[i].Add(0);
            }

            // Calculte total inter-cluster distance
            double travelTotalDistance = calculateClusterTravelDistance(clusterRouteForVehicle, this.clustersDistanceMatrix);

            // Set solution
            ClusterSolution solution = new ClusterSolution(clusterRouteForVehicle, travelTotalDistance);

            // Return solution
            return solution;
        }
    }

    
}
