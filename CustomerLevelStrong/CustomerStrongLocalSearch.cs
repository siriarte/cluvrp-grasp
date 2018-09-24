using System;
using System.Collections.Generic;
using System.Linq;

namespace cluvrp_grasp
{

    class CustomerStrongLocalSearch
    {
        // Attributes
        public int maxIterationsWithoutImprovementTwoOpt { get; set; }
        public int maxIterationsWithoutImprovementRelocate { get; set; }
        public int maxIterationsWithoutImprovementExchange { get; set; }
        public int maxIterationsWithoutImprovementSwapCustomers { get; set; }
        public CluVRPSolution solution { get; set; }
        public CluVRPInstance instance { get; set; }

        // Constructor
        public CustomerStrongLocalSearch(CluVRPSolution solution, 
            CluVRPInstance instance,
            int maxIterationsWithoutImprovementTwoOpt = 100,
            int maxIterationsWithoutImprovementRelocate = 100,
            int maxIterationsWithoutImprovementExchange = 100,
            int maxIterationsWithoutImprovementSwapCustomers = 100)
        {
            this.solution = solution;
            this.instance = instance;
            this.maxIterationsWithoutImprovementTwoOpt = maxIterationsWithoutImprovementTwoOpt;
            this.maxIterationsWithoutImprovementRelocate = maxIterationsWithoutImprovementRelocate;
            this.maxIterationsWithoutImprovementExchange = maxIterationsWithoutImprovementExchange;
            this.maxIterationsWithoutImprovementSwapCustomers = maxIterationsWithoutImprovementSwapCustomers;
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
                for (int clusterIt = 1; clusterIt  + 1 < clusterRoute.Length; clusterIt++)
                {
                    // Best distance 
                    double bestDistance = Functions.calculateInOutAndPathDistance(customersCircuit[vehicle], clusterIt, instance.customersDistanceMatrix);

                    // Main cycle
                    int iteration = 0;
                    while (iteration < maxIterationsWithoutImprovementTwoOpt)
                    {
                        // For every customer path of a cluster
                        List<int> route = clusterRoute[clusterIt];
                        int maxW = route.Count();

                        // Starting from second customer (avoid depot)
                        for (int i = 0; i + 1< maxW; i++)
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
                                double newDistance = Functions.calculateInOutAndPathDistance(customersCircuit[vehicle], clusterIt, instance.customersDistanceMatrix);

                                // If distance if better
                                if (newDistance + 0.000001 < bestDistance)
                                {                                    
                                    // Update best distance
                                    this.solution.vehiculeRouteDistance[vehicle] = Functions.calculateTotalTravelDistance(customersCircuit, instance.customersDistanceMatrix, vehicle);
                                    bestDistance = newDistance;

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
                for (int clusterIt = 1; clusterIt + 1 < numberOfClusters; clusterIt++)
                {
                     // Get the customer route of the cluste
                    List<int> route = solution.customersPaths[vehicle][clusterIt];

                    // Main cycle
                    int iteration = 0;
                    while (iteration < maxIterationsWithoutImprovementRelocate)
                    {
                        // For each i-customer
                        for (int i = 0; i < route.Count; i++)
                        {
                            // For each j-customer
                            for (int j = 0 ; j < route.Count; j++)
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
            if (!(prev_customer_i != 0 && prev_customer_j != 0 && prev_customer_i != lastIndex && prev_customer_j != lastIndex &&
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
            if (newDistance + 0.000001 < solution.vehiculeRouteDistance[vehicle])
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
                for (int cluster = 1; cluster + 1 < solution.customersPaths[vehicle].Length; cluster++)
                {
                    // Main Cyvcle
                    int iteration = 0;
                    while (iteration < maxIterationsWithoutImprovementExchange)
                    {
                        // For each i-customer except the initial one
                        for (int i = 0; i < solution.customersPaths[vehicle][cluster].Count; i++)
                        {
                            // For each j-customer except the last one
                            for (int j = 0; j < solution.customersPaths[vehicle][cluster].Count; j++)
                            {
                                // If are the same customer or the depot dont do anything
                                if ((i == j) || ((i == 0 && j == solution.customersPaths[vehicle][cluster].Count - 1) ||
                                    (i == solution.customersPaths[vehicle][cluster].Count - 1 && j == 0)))
                                    continue;

                                // If perform realocate success 
                                if (exchange(vehicle, cluster, i, j))
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
        private bool exchange(int vehicle, int cluster, int i, int j)
        {
            // Get route
            List<int> route = solution.customersPaths[vehicle][cluster];

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
            if (newDistance + 0.000001 < this.solution.vehiculeRouteDistance[vehicle])
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
        
        // Swap Algorithm
        public void swapCustomers()
        {
            // For each vehicle
            for (int vehicle = 0; vehicle < solution.customersPaths.Length; vehicle++)
            {
                // If vehicle has only 1 cluster doesn't try
                if (solution.customersPaths[vehicle].Length < 2) continue;

                // Calculate path distance including In and Out of cluster
                double bestDistance = solution.vehiculeRouteDistance[vehicle];

                // Main cycle
                int iterator = 0;
                while (iterator < maxIterationsWithoutImprovementSwapCustomers)
                {
                    // If solution improves try with one more iteration
                    bool solutionImproves = false;

                    // For each cluster
                    for (int clusterIt = 1; clusterIt < solution.customersPaths[vehicle].Length - 1; clusterIt++)
                    {
                        // Select cluste random
                        int cluster = clusterIt;

                        // For random order on iterations
                        List<int> rndPosition = new List<int>();
                        for (int i = 0; i < solution.customersPaths[vehicle][cluster].Count; i++) rndPosition.Add(i);
                        Functions.Shuffle<int>(new Random(), rndPosition);
                       
                        // For each customer
                        for (int customerIt_1 = 0; customerIt_1 < solution.customersPaths[vehicle][cluster].Count; customerIt_1++)
                        {
                            // Set customer1
                            int customerIt1 = rndPosition[customerIt_1];

                            // Against all customer on the path
                            for (int customerIt_2 = 0; customerIt_2 < solution.customersPaths[vehicle][cluster].Count; customerIt_2++)
                            {
                                // Set customer1
                                int customerIt2 = rndPosition[customerIt_2];

                                // No swap for same customers
                                if (customerIt1 == customerIt2)
                                {
                                    continue;
                                }

                                // If is border case calculate all path plus diff in border
                                if (customerIt1 == 0 || customerIt1 == solution.customersPaths[vehicle][cluster].Count - 1 ||
                                    customerIt2 == 0 || customerIt2 == solution.customersPaths[vehicle][cluster].Count - 1)
                                {
                                    // Calculate section old distance
                                    double clusterOldDistance = Functions.calculateInOutAndPathDistance(solution.customersPaths[vehicle], cluster, instance.customersDistanceMatrix);

                                    // Perform Swap
                                    Functions.Swap(solution.customersPaths[vehicle][cluster], customerIt1, customerIt2);

                                    // Calculate section new distance
                                    double clusterNewDistance = Functions.calculateInOutAndPathDistance(solution.customersPaths[vehicle], cluster, instance.customersDistanceMatrix);

                                    // Calculate vehicle new distance
                                    double newDistance = bestDistance - clusterOldDistance + clusterNewDistance;

                                    // If solution not improve swap back
                                    if (newDistance + 0.000001 < bestDistance)
                                    {
                                        // Update distance
                                        solution.vehiculeRouteDistance[vehicle] = newDistance;
                                        bestDistance = newDistance;

                                        // Reset iterator
                                        solutionImproves = true;
                                        iterator = 0;
                                    }
                                    else
                                    {
                                        // Perform Swap back
                                        Functions.Swap(solution.customersPaths[vehicle][cluster], customerIt1, customerIt2);                                    
                                    }
                                }
                                // If is not a boerder case only calculate the diff on the swap
                                else
                                {
                                    // Calculate old diff distance
                                    int customer1 = solution.customersPaths[vehicle][cluster][customerIt1];
                                    int customer2 = solution.customersPaths[vehicle][cluster][customerIt2];
                                    int customer1_next = solution.customersPaths[vehicle][cluster][customerIt1 + 1];
                                    int customer1_last = solution.customersPaths[vehicle][cluster][customerIt1 + -1];
                                    int customer2_next = solution.customersPaths[vehicle][cluster][customerIt2 + 1];
                                    int customer2_last = solution.customersPaths[vehicle][cluster][customerIt2 - 1];

                                    double oldDiffCustomer1 = instance.customersDistanceMatrix[customer1_last][customer1] +
                                       instance.customersDistanceMatrix[customer1][customer1_next];
                                    double oldDiffCustomer2 = instance.customersDistanceMatrix[customer2_last][customer2] +
                                        instance.customersDistanceMatrix[customer2][customer2_next];

                                    // If one is the next of the other
                                    if (customerIt2 - customerIt1 == 1)
                                    {
                                        customer1_next = customer1;
                                        customer2_last = customer2;
                                    }
                                    else if (customerIt1 - customerIt2 == 1)
                                    {
                                        customer2_next = customer2;
                                        customer1_last = customer1;
                                    }

                                    // Calculate new diff distance 
                                    double newDiffCustomer1 = instance.customersDistanceMatrix[customer2_last][customer1] +
                                       instance.customersDistanceMatrix[customer1][customer2_next];
                                    double newDiffCustomer2 = instance.customersDistanceMatrix[customer1_last][customer2] +
                                       instance.customersDistanceMatrix[customer2][customer1_next];

                                    // Calculate total new distance
                                    double newDistance = bestDistance - (oldDiffCustomer1 + oldDiffCustomer2) + (newDiffCustomer1 + newDiffCustomer2);

                                    // If solution not improve swap back
                                    if (newDistance + 0.000001 < bestDistance)
                                    {
                                        // Perform Swap
                                        Functions.Swap(solution.customersPaths[vehicle][cluster], customerIt1, customerIt2);

                                        // Update distance
                                        solution.vehiculeRouteDistance[vehicle] = solution.vehiculeRouteDistance[vehicle] - bestDistance + newDistance;
                                        bestDistance = newDistance;

                                        // Reset iterator
                                        solutionImproves = true;
                                        iterator = 0;
                                    }

                                }// End else not border case

                                // To back to cluster
                                if (solutionImproves) break;

                            }// End for customer1

                            // To back to cluster
                            if (solutionImproves) break;
                        }// End for customer2

                        // To back to cluster
                        if (solutionImproves) break;

                    } // End for cluster

                    // If not improves increase iterator
                    if (!solutionImproves) iterator++;

                } // End main iterations
            }// End for vehicle

        }

    }
}
