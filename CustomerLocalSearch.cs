using System;
using System.Collections.Generic;
using System.Linq;

namespace cluvrp_grasp
{

    class CustomerLocalSearch
    {
        // Attributes
        public int maxIterationsWithoutImprovementTwoOpt { get; set; }
        public int maxIterationsWithoutImprovementRelocate { get; set; }
        public int maxIterationsWithoutImprovementExchange { get; set; }
        public CluVRPSolution solution { get; set; }
        public CluVRPInstance instance { get; set; }

        // Constructor
        public CustomerLocalSearch(CluVRPSolution solution, 
            CluVRPInstance instance,
            int maxIterationsWithoutImprovementTwoOpt = 100,
            int maxIterationsWithoutImprovementRelocate = 100,
            int maxIterationsWithoutImprovementExchange = 100)
        {
            this.solution = solution;
            this.instance = instance;
            this.maxIterationsWithoutImprovementTwoOpt = maxIterationsWithoutImprovementTwoOpt;
            this.maxIterationsWithoutImprovementRelocate = maxIterationsWithoutImprovementRelocate;
            this.maxIterationsWithoutImprovementExchange = maxIterationsWithoutImprovementExchange;
        }

        // TwoOpt local-search 
        public void twoOpt()
        {
            // Init variables
            List<int>[][] customersCircuit = solution.customersPaths;
            int numberOfVehicles = customersCircuit.Length;
          
            // For each vehicle route
            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                // Calculate the complete travel distance by vehicle
                List<int>[] clusterRoute = customersCircuit[vehicle];
             
                // For every cluster on the route
                for (int clusterIt = 0; clusterIt < clusterRoute.Length; clusterIt++)
                {
                    // Main cycle
                    int iteration = 0;
                    while (iteration < maxIterationsWithoutImprovementTwoOpt)
                    {
                        // For every customer path of a cluster
                        List<int> route = clusterRoute[clusterIt];
                        int maxW = route.Count();

                        // Starting from second customer (avoid depot)
                        for (int i = 1; i + 1< maxW; i++)
                        {
                            // Until the last one on the path
                            for (int j = i + 1; j < maxW; j++)
                            {
                                // Backup original route
                                List<int> oldRoute = customersCircuit[vehicle][clusterIt];

                                // New route create by two-opt-swap between customer i and j 
                                List<int> newRoute = twoOptSwap(customersCircuit[vehicle][clusterIt], i, j);

                                // Assign the new route to the circuit
                                customersCircuit[vehicle][clusterIt] = newRoute;

                                // Calculate the new distance
                                double newDistance = Functions.calculateTotalTravelDistance(customersCircuit, instance.customersDistanceMatrix, vehicle);

                                // If distance if better
                                if (newDistance + 0.01 < this.solution.vehiculeRouteDistance[vehicle])
                                {
                                    // Update best distance
                                    this.solution.vehiculeRouteDistance[vehicle] = newDistance;
                                    
                                    // Restart iterator
                                    iteration = 0;
                                }
                                else
                                {
                                    // Restore old route to the circuit
                                    customersCircuit[vehicle][clusterIt] = oldRoute;
                                }
                            } // end for j
                        } // end for i

                        // Increase iterator
                        iteration++;

                    } // End for clusterIt
                } // End for iterator
            } // End for vehicle

            // End
            return;
        }
        
        // TwoOpt Algorithm
        public List<int> twoOptSwap(List<int> route, int i, int k)
        {
            List<int> newRoute = route.GetRange(0, i);
            int reverseSize = k - i + 1;
            List<int> reverseRoute = route.GetRange(i, reverseSize);
            reverseRoute.Reverse();
            int restSize = route.Count - (k + 1);
            List<int> endRoute = route.GetRange(k + 1, restSize);
            newRoute.AddRange(reverseRoute);
            newRoute.AddRange(endRoute);
            return newRoute;
        }

        // Relocate local-search
        public void relocate()
        {
            // Init variables
            int numberOfVehicles = solution.customersPaths.Length;
            
            // For each vehicle
            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                // Get number of cluster on the vehicle route 
                int numberOfClusters = solution.customersPaths[vehicle].Length;

                // For each cluster
                for (int clusterIt = 0; clusterIt < numberOfClusters; clusterIt++)
                {                    
                    // Get the customer route of the cluste
                    List<int> route = solution.customersPaths[vehicle][clusterIt];

                    // Main cycle
                    int iteration = 0;
                    while (iteration < maxIterationsWithoutImprovementRelocate)
                    {
                        // For each i-customer
                        for (int i = 1; i + 1 < route.Count; i++)
                        {
                            // For each j-customer
                            for (int j = 1 ; j + 1 < route.Count; j++)
                            {
                                // If are the same customer or the depot dont do anything
                                if ((i == j) || ((i == 0 && j == route.Count - 1) ||
                                    (i == route.Count - 1 && j == 0)))
                                {
                                    continue;
                                }

                                // If perform realocate success 
                                if (relocate(vehicle, clusterIt, i, j))
                                {
                                    // Restart iterator
                                    iteration = 0;
                                }
                            } // End for j
                        } // End for i

                        // Increase iterator
                        iteration++;

                    } // End for While 
                } // End for clusterIt
            } // End for vehicle
        }

        // Relocate Algorithm
        public bool relocate(int vehicle, int cluster, int i, int j)
        {
            // Get customer route
            List<int> route = solution.customersPaths[vehicle][cluster];
            int lastIndex = solution.customersPaths[vehicle][cluster].Count - 1;

            // To be on the way
            var prev_customer_i = i - 1 == -1 ? route.Count - 1 : i - 1;
            var prev_customer_j = j - 1 == -1 ? route.Count - 1 : j - 1;
            var next_customer_i = i + 1 == route.Count ? 0 : i + 1;
            var next_customer_j = j + 1 == route.Count ? 0 : j + 1;

            // Increase
            if (i < j)
                prev_customer_j++;
            else
                next_customer_j--;

            // Doesn't use init or final customer
            if(!(prev_customer_i != 0 && prev_customer_j != 0 && prev_customer_i != lastIndex && prev_customer_j != lastIndex &&
                next_customer_i != 0 && next_customer_j != 0 && next_customer_i != lastIndex && next_customer_j != lastIndex))
            {
                return false;
            }

            // Set variables to calculate new distance
            double _a = instance.customersDistanceMatrix[route[prev_customer_i]][route[i]];
            double _b = instance.customersDistanceMatrix[route[i]][route[next_customer_i]];
            double _C = instance.customersDistanceMatrix[route[prev_customer_i]][route[next_customer_i]];
            double _A = instance.customersDistanceMatrix[route[prev_customer_j]][route[i]];
            double _B = instance.customersDistanceMatrix[route[i]][route[next_customer_j]];
            double _c = instance.customersDistanceMatrix[route[prev_customer_j]][route[next_customer_j]];

            // Calculate new distance
            var newDistance = solution.vehiculeRouteDistance[vehicle] - _a - _b + _C + _A + _B - _c;

            // If new distance is better
            if (newDistance + 0.01 < solution.vehiculeRouteDistance[vehicle])
            {
                // Perform realocate
                int customer = route[i];
                route.RemoveAt(i);
                route.Insert(j, customer);

                 // Update new distances
                 solution.vehiculeRouteDistance[vehicle] = newDistance;
                 return true;
            }

            // Relocate is not posible
            return false;
        }

        // Exchange local-search 
        public void exchange()
        {
            // Get number of vehicles
            int numberOfVehicles = solution.customersPaths.Length;
              
            // For each vehicle
            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                // For each customer route
                for (int cluster = 0; cluster < solution.customersPaths[vehicle].Length; cluster++)
                {
                    // Main Cyvcle
                    int iteration = 0;
                    while (iteration < maxIterationsWithoutImprovementExchange)
                    {
                        // Get the vehicle route
                        List<int> route = solution.customersPaths[vehicle][cluster];

                        // For each i-customer except the initial one
                        for (int i = 1; i < route.Count; i++)
                        {
                            // For each j-customer except the last one
                            for (int j = 1; j + 1 < route.Count; j++)
                            {
                                // If are the same customer or the depot dont do anything
                                if ((i == j) || ((i == 0 && j == route.Count - 1) ||
                                    (i == route.Count - 1 && j == 0)))
                                    continue;

                                // If perform realocate success 
                                if (exchange(route, vehicle, cluster, i, j))
                                {
                                    // Restart iterator
                                    iteration = 0;
                                }
                            } // End for i
                        } // End for j

                        // Increase iterator
                        iteration++;

                    } // End while
                } // End for cluster 
            } // End for vehicle

            //End
            return;
        }

        // Exchange Algorithm
        bool exchange(List<int> route, int vehicle, int cluster, int i, int j)
        {
            // Get last customer index on the route
            int lastIndex = route.Count - 1;

            // To be on the route
            int prev_customer_i = i - 1 == -1 ? route.Count - 1 : i - 1;
            int prev_customer_j = j - 1 == -1 ? route.Count - 1 : j - 1;
            int next_customer_i = i + 1 == route.Count ? 0 : i + 1;
            int next_customer_j = j + 1 == route.Count ? 0 : j + 1;

            // Doesn't use init or final customer
            if (!(prev_customer_i != 0 && prev_customer_j != 0 && prev_customer_i != lastIndex && prev_customer_j != lastIndex &&
                next_customer_i != 0 && next_customer_j != 0 && next_customer_i != lastIndex && next_customer_j != lastIndex))
            {
                return false;
            }

            // Old distances
            double distance_i_left = instance.customersDistanceMatrix[route[prev_customer_i]][route[i]];
            double distance_i_right = instance.customersDistanceMatrix[route[i]][route[next_customer_i]];
            double distance_j_left = instance.customersDistanceMatrix[route[prev_customer_j]][route[j]];
            double distance_j_right = instance.customersDistanceMatrix[route[j]][route[next_customer_j]];

            // New distances
            double new_distance_i_left, new_distance_i_right;
            double new_distance_j_left, new_distance_j_right;

            // If i is next customer j
            if (i == next_customer_j)
            {
                // Calculate new distance for i
                new_distance_i_left = instance.customersDistanceMatrix[route[prev_customer_j]][route[i]];
                new_distance_i_right = instance.customersDistanceMatrix[route[i]][route[j]];

                // Calculate new distance for j
                new_distance_j_right = instance.customersDistanceMatrix[route[j]][route[next_customer_i]];
                new_distance_j_left = instance.customersDistanceMatrix[route[j]][route[i]];
            }
            else if (j == next_customer_i)
            {
                // Calculate new distance for i
                new_distance_i_right = instance.customersDistanceMatrix[route[i]][route[next_customer_j]];
                new_distance_i_left = instance.customersDistanceMatrix[route[j]][route[i]];

                // Calculate new distance for j
                new_distance_j_left = instance.customersDistanceMatrix[route[prev_customer_i]][route[j]];
                new_distance_j_right = instance.customersDistanceMatrix[route[i]][route[j]];
            }
            else
            {
                // Calculate new distance for i
                new_distance_i_left = instance.customersDistanceMatrix[route[prev_customer_j]][route[i]];
                new_distance_i_right = instance.customersDistanceMatrix[route[i]][route[next_customer_j]];

                // Calculate new distance for j
                new_distance_j_left = instance.customersDistanceMatrix[route[prev_customer_i]][route[j]];
                new_distance_j_right = instance.customersDistanceMatrix[route[j]][route[next_customer_i]];
            }

            // Calculate new total distance
            double newDistance = solution.vehiculeRouteDistance[vehicle] - distance_i_left - distance_i_right -
                distance_j_left - distance_j_right + new_distance_i_left + new_distance_i_right +
                new_distance_j_left + new_distance_j_right;

            // If new distance is better
            if (newDistance + 0.01 < this.solution.vehiculeRouteDistance[vehicle])
            {
                // Perform exchange
                int customer = route[i];
                route[i] = route[j];
                route[j] = customer;

                // Update distance
                solution.vehiculeRouteDistance[vehicle] = newDistance;
                
                // Exchange was possible
                return true;
               
            } // End if distance is better

            // Perform exchange is not possible
            return false;
        }
           
    }
}
