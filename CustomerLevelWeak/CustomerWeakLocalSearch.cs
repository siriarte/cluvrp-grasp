using System;
using System.Collections.Generic;
using System.Linq;

namespace cluvrp_grasp
{

    class CustomerWeakLocalSearch
    {
        // Attributes
        public int maxIterationsWithoutImprovementTwoOpt { get; set; }
        public int maxIterationsWithoutImprovementRelocate { get; set; }
        public int maxIterationsWithoutImprovementExchange { get; set; }
        public int maxIterationsWithoutImprovementSwapCustomers { get; set; }
        public CluVRPSolution solution { get; set; }
        public CluVRPInstance instance { get; set; }

        // Constructor
        public CustomerWeakLocalSearch(CluVRPSolution solution,
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
            List<int>[] customersCircuit = solution.customersWeakRoute;
            int numberOfVehicles = customersCircuit.Length;

            // For each vehicle route
            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                // Best distance 
                double bestDistance = Functions.calculateCustomerTravelDistance(customersCircuit[vehicle], instance.customersDistanceMatrix);

                // Main cycle
                int iteration = 0;
                while (iteration < maxIterationsWithoutImprovementTwoOpt)
                {
                    // For every customer path of a cluster
                    List<int> route = customersCircuit[vehicle];
                    int maxW = route.Count();

                    // Starting from second customer (avoid depot)
                    for (int i = 0; i < maxW; i++)
                    {
                        // Until the last one on the path
                        for (int j = i + 1; j < maxW; j++)
                        {
                            // Backup original route
                            List<int> oldRoute = customersCircuit[vehicle];

                            // New route create by two-opt-swap between customer i and j 
                            List<int> newRoute = twoOptSwap(customersCircuit[vehicle], i, j);

                            // Assign the new route to the circuit
                            customersCircuit[vehicle] = newRoute;

                            // Calculate the new distance
                            double newDistance = Functions.calculateCustomerTravelDistance(customersCircuit[vehicle], instance.customersDistanceMatrix);

                            // If distance if better
                            if (newDistance + 0.000001 < bestDistance && isValidRoute(newRoute))
                            {
                                // Update best distance
                                this.solution.vehiculeRouteDistance[vehicle] = newDistance;
                                bestDistance = newDistance;

                                // Restart iterator
                                iteration = 0;
                            }
                            else
                            {
                                // Restore old route to the circuit
                                customersCircuit[vehicle] = oldRoute;
                            }
                        } // end for j
                    } // end for i

                    // Increase iterator
                    iteration++;
                
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
            int numberOfVehicles = solution.customersWeakRoute.Length;

            // For each vehicle
            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {                    
                // Get the customer route of the cluste
                List<int> route = solution.customersWeakRoute[vehicle];

                // Main cycle
                int iteration = 0;
                while (iteration < maxIterationsWithoutImprovementRelocate)
                {
                    // For each i-customer
                    for (int i = 0; i < route.Count; i++)
                    {
                        // For each j-customer
                        for (int j = 0; j < route.Count; j++)
                        {
                            // If are the same customer or the depot dont do anything
                            if ((i == j) || ((i == 0 && j == route.Count - 1) ||
                                (i == route.Count - 1 && j == 0)))
                            {
                                continue;
                            }

                            // If perform realocate success 
                            if (relocate(vehicle, i, j))
                            {
                                // Restart iterator
                                iteration = 0;
                            }
                        } // End for j
                    } // End for i

                    // Increase iterator
                    iteration++;

                } // End for While 
            } // End for vehicle
        }

        // Relocate Algorithm
        public bool relocate(int vehicle, int i, int j)
        {
            // Get customer route
            List<int> route = solution.customersWeakRoute[vehicle];
            int lastIndex = solution.customersWeakRoute[vehicle].Count - 1;

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
            int lastCustomerIndex = route.Count - 1;
            if (newDistance + 0.000001 < solution.vehiculeRouteDistance[vehicle] && 
                i != 0 && i != lastCustomerIndex && j != 0 && j != lastCustomerIndex)
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
            int numberOfVehicles = solution.customersWeakRoute.Length;

            // For each vehicle
            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                // Main Cyvcle
                int iteration = 0;
                while (iteration < maxIterationsWithoutImprovementExchange)
                {
                    // For each i-customer except the initial one
                    for (int i = 1; i + 1 < solution.customersWeakRoute[vehicle].Count; i++)
                    {
                        // For each j-customer except the last one
                        for (int j = 1; j + 1 < solution.customersWeakRoute[vehicle].Count; j++)
                        {
                            // If are the same customer or the depot dont do anything
                            if ((i == j) || ((i == 0 && j == solution.customersWeakRoute[vehicle].Count - 1) ||
                                (i == solution.customersWeakRoute[vehicle].Count - 1 && j == 0)))
                                continue;

                            // If perform realocate success 
                            if (exchange(vehicle, i, j))
                            {
                                // Restart iterator
                                iteration = 0;
                            }
                        } // End for i
                    } // End for j

                    // Increase iterator
                    iteration++;

                    } // End while
            } // End for vehicle

            //End
            return;
        }

        // Exchange Algorithm
        private bool exchange(int vehicle, int i, int j)
        {
            // Get route
            List<int> route = solution.customersWeakRoute[vehicle];

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
            int lastCustomerIndex = route.Count - 1;
            if (newDistance + 0.000001 < this.solution.vehiculeRouteDistance[vehicle] &&
                i != 0 && i != lastCustomerIndex && j != 0 && j != lastCustomerIndex)
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
            for (int vehicle = 0; vehicle < solution.customersWeakRoute.Length; vehicle++)
            {                   
                // Calculate path distance including In and Out of cluster
                double bestDistance = solution.vehiculeRouteDistance[vehicle];

                // For random order on iterations
                List<int> rndPosition = new List<int>();
                for (int i = 0; i < solution.customersWeakRoute[vehicle].Count; i++) rndPosition.Add(i);

                // Main cycle
                int iterator = 0;
                while (iterator < maxIterationsWithoutImprovementSwapCustomers)
                {

                    // Suffle list positions
                    Functions.Shuffle<int>(new Random(), rndPosition);

                    // If solution improves jump and start again with the same vehicle
                    bool solutionImproves = false;

                    // For each customer except depot (start and end)
                    for (int customerIt_1 = 0; customerIt_1 < solution.customersWeakRoute[vehicle].Count; customerIt_1++)
                    {
                        // Against all customer on the path (start and end)
                        for (int customerIt_2 = 0; customerIt_2 < solution.customersWeakRoute[vehicle].Count; customerIt_2++)
                        {
                            // Select rnd position for iterators
                            int customerIt1 = rndPosition[customerIt_1];
                            int customerIt2 = rndPosition[customerIt_2];

                            // No swap for same customers or depot
                            if (customerIt_1 == customerIt_2 || 
                                customerIt1 == 0 || customerIt1 == solution.customersWeakRoute[vehicle].Count - 1 ||
                                customerIt2 == 0 || customerIt2 == solution.customersWeakRoute[vehicle].Count - 1)
                            {
                                continue;
                            }
                            
                            // Calculate old diff distance
                            int customer1 = solution.customersWeakRoute[vehicle][customerIt1];
                            int customer2 = solution.customersWeakRoute[vehicle][customerIt2];
                            int customer1_next = solution.customersWeakRoute[vehicle][customerIt1 + 1];
                            int customer1_last = solution.customersWeakRoute[vehicle][customerIt1 + -1];
                            int customer2_next = solution.customersWeakRoute[vehicle][customerIt2 + 1];
                            int customer2_last = solution.customersWeakRoute[vehicle][customerIt2 - 1];

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
                            if (newDistance + 0.00001 <= bestDistance)
                            {
                                // Perform Swap
                                Functions.Swap(solution.customersWeakRoute[vehicle], customerIt1, customerIt2);

                                // Update distance
                                bestDistance = newDistance;
                                solution.vehiculeRouteDistance[vehicle] = newDistance;

                                // Reset irator
                                iterator = 0;
                                solutionImproves = true;
                                break;
                            }
                        }// End for customer1

                        // To start from vehicle again
                        if (solutionImproves) break;

                    }// End for customer2

                    // If solution not improve continue with next cluster
                    if (!solutionImproves) iterator++;
                }
            }// End for vehicle

            // End
            return;
        }

        // Check if route start and end on depot
        bool isValidRoute(List<int> route)
        {
            if (route.Count < 2) return false;
            return (route[0] == 1 && route[route.Count - 1] == 1);
        }

    }
}
