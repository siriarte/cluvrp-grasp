﻿using System;
using System.Collections.Generic;
using System.Linq;


namespace cluvrp_grasp
{
          
    class ClusterLocalSearch
    {
        // Attributes
        public int maxIterationsWithoutImprovementTwoOpt { get; set; }
        public int maxIterationsWithoutImprovementRelocate { get; set; }
        public int maxIterationsWithoutImprovementExchange { get; set; }
        public int maxIterationsWithoutImprovementIVRS { get; set; }
        public int maxIterationsWithoutImprovementIVRC { get; set; }
        public CluVRPSolution solution { get; set; }
        public CluVRPInstance instance;

        // Constructor
        public ClusterLocalSearch(CluVRPSolution solution,
            CluVRPInstance instance,
            int maxIterationsWithoutImprovementTwoOpt = 100,
            int maxIterationsWithoutImprovementRelocate = 100,
            int maxIterationsWithoutImprovementExchange = 100,
            int maxIterationsWithoutImprovementIVRS = 100,
            int maxIterationsWithoutImprovementIVRC = 100)
        {
            this.solution = solution;
            this.instance = instance;
            this.maxIterationsWithoutImprovementTwoOpt = maxIterationsWithoutImprovementTwoOpt;
            this.maxIterationsWithoutImprovementRelocate = maxIterationsWithoutImprovementRelocate;
            this.maxIterationsWithoutImprovementExchange = maxIterationsWithoutImprovementExchange;
            this.maxIterationsWithoutImprovementIVRS = maxIterationsWithoutImprovementIVRS;
            this.maxIterationsWithoutImprovementIVRC = maxIterationsWithoutImprovementIVRC;
        }

        // TwoOpt local-search 
        public void twoOpt()
        {
            // Init variables
            List<int>[] routeForVehicule = solution.clusterRouteForVehicule;
            int numberOfVehicles = routeForVehicule.Length;
            double[] bestDistance = new double[numberOfVehicles];

            // For each vehicle
            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++) {

                // Get the vehicle route
                List<int> route = routeForVehicule[vehicle];
                int maxW = route.Count();
                bestDistance[vehicle] = Functions.calculateClusterTravelDistanceForVehicle(route, instance.clustersDistanceMatrix);

                // Main cycle
                int iterator = 0;
                while (iterator < maxIterationsWithoutImprovementTwoOpt)
                {
                    // Starting from second customer (avoid depot)
                    for (int i = 1; i + 1 < maxW; i++)
                    {
                        // Until the last one on the path
                        for (int j = i + 1; j < maxW; j++)
                        {
                            // New route create by two-opt-swap between customer i and j 
                            List<int> newRoute = twoOptSwap(route, i, j);

                            // Calculate distance of the new route
                            double newDistance = Functions.calculateClusterTravelDistanceForVehicle(newRoute, instance.clustersDistanceMatrix);

                            // If distance if better and route is valid
                            if (newDistance + 0.01 < bestDistance[vehicle] && Functions.isValidClusterRoute(newRoute))
                            {
                                // Update with new route and distance
                                routeForVehicule[vehicle] = newRoute;
                                bestDistance[vehicle] = newDistance;

                                // Reset iterator
                                iterator = 0;
                            }
                        } // End for j
                    } // End for i

                    // Incresase iterator
                    iterator++;

                } // End While
            } // End for vehicle

            // Update the total cluster route distance (the sum all vehicle cluster distances)
            this.solution.totalClusterRouteDistance = bestDistance.Sum();
        }

        // TwoOpt-Swap Algorithm
        public List<int> twoOptSwap(List<int> route, int i, int k)
        {
            List<int> newRoute = route.GetRange(0, i);
            int reverseSize = k - i + 1;
            List<int> reverseRoute = route.GetRange(i, reverseSize);
            reverseRoute.Reverse();
            int restSize = route.Count - (k + 1);
            List<int> endRoute = route.GetRange(k+1, restSize);
            newRoute.AddRange(reverseRoute);
            newRoute.AddRange(endRoute);
            return newRoute;
        }

        // Relocate local-search
        public void relocate()
        {
            // Init variables
            int numberOfVehicles = solution.clusterRouteForVehicule.Length;

            // For each vehicle
            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                // Get cluster route for vehicle
                List<int> route = solution.clusterRouteForVehicule[vehicle];

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
                            // If is the same customer or the depot dont do anything
                            if ((i == j) || ((i == 0 && j == route.Count - 1) ||
                                (i == route.Count - 1 && j == 0)))
                                continue;

                            // If perform recolate success 
                            if (relocate(route, vehicle, i, j))
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
        public bool relocate(List<int> route, int vehicle, int i, int j)
        {
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

            // Set variables to calculate new distance
            double _a = instance.clustersDistanceMatrix[route[prev_customer_i]][route[i]];
            double _b = instance.clustersDistanceMatrix[route[i]][route[next_customer_i]];
            double _C = instance.clustersDistanceMatrix[route[prev_customer_i]][route[next_customer_i]];
            double _A = instance.clustersDistanceMatrix[route[prev_customer_j]][route[i]];
            double _B = instance.clustersDistanceMatrix[route[i]][route[next_customer_j]];
            double _c = instance.clustersDistanceMatrix[route[prev_customer_j]][route[next_customer_j]];

            // Calculate new distance
            var newDistance = solution.totalClusterRouteDistance - _a - _b + _C + _A + _B - _c;

            // If new distance is better
            if (newDistance + 0.01 < solution.totalClusterRouteDistance)
            {
                // Perform realocate
                int customer = route[i];
                route.RemoveAt(i);
                route.Insert(j, customer);

                // If route is valid
                if (Functions.isValidClusterRoute(route))
                {
                    // Update new distances
                    solution.totalClusterRouteDistance = newDistance;
                    return true;
                }
                else
                {   
                    // Back to the old route
                    route.RemoveAt(j);
                    route.Insert(i, customer);
                }
            }
            
            // Relocate is not posible
            return false;
        }

        // Exchange local-search
        public void exchange()
        {
            // Get number of vehicles
            int numberOfVehicles = solution.clusterRouteForVehicule.Length;

            // For each vehicle
            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                // Get cluster route for vehicle
                List<int> route = solution.clusterRouteForVehicule[vehicle];

                // Main cycle
                int iteration = 0;
                while (iteration < maxIterationsWithoutImprovementExchange)
                {
                    // For each i-customer
                    for (int i = 0; i < route.Count; i++)
                    {
                        // For each j-customer
                        for (int j = 0; j < route.Count; j++)
                        {
                            // If is the same customer or the depot dont do anything
                            if ((i == j) || ((i == 0 && j == route.Count - 1) ||
                                (i == route.Count - 1 && j == 0)))
                                continue;

                            // If perform exchange success 
                            if (exchange(route, vehicle, i, j))
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

        // Exchange Algorithm
        bool exchange(List<int> route, int vehicle, int i, int j)
        {     
            // To be on the route
            int prev_customer_i = i - 1 == -1 ? route.Count - 1 : i - 1;
            int prev_customer_j = j - 1 == -1 ? route.Count - 1 : j - 1;
            int next_customer_i = i + 1 == route.Count ? 0 : i + 1;
            int next_customer_j = j + 1 == route.Count ? 0 : j + 1;

            // Old distances
            var distance_i_left = instance.clustersDistanceMatrix[route[prev_customer_i]][route[i]];
            var distance_i_right = instance.clustersDistanceMatrix[route[i]][route[next_customer_i]];
            var distance_j_left = instance.clustersDistanceMatrix[route[prev_customer_j]][route[j]];
            var distance_j_right = instance.clustersDistanceMatrix[route[j]][route[next_customer_j]];
            
            // New distances
            double new_distance_i_left, new_distance_i_right;
            double new_distance_j_left, new_distance_j_right;

            // If i is next customer j
            if (i == next_customer_j)
            {
                // Calculate new distance for i
                new_distance_i_left = instance.clustersDistanceMatrix[route[prev_customer_j]][route[i]];
                new_distance_i_right = instance.clustersDistanceMatrix[route[i]][route[j]];

                // Calculate new distance for j
                new_distance_j_right = instance.clustersDistanceMatrix[route[j]][route[next_customer_i]];
                new_distance_j_left = instance.clustersDistanceMatrix[route[j]][route[i]];
            }
            else if (j == next_customer_i)
            {
                // Calculate new distance for i
                new_distance_i_right = instance.clustersDistanceMatrix[route[i]][route[next_customer_j]];
                new_distance_i_left = instance.clustersDistanceMatrix[route[j]][route[i]];

                // Calculate new distance for j
                new_distance_j_left = instance.clustersDistanceMatrix[route[prev_customer_i]][route[j]];
                new_distance_j_right = instance.clustersDistanceMatrix[route[i]][route[j]];
            }
            else
            {
                // Calculate new distance for i
                new_distance_i_left = instance.clustersDistanceMatrix[route[prev_customer_j]][route[i]];
                new_distance_i_right = instance.clustersDistanceMatrix[route[i]][route[next_customer_j]];

                // Calculate new distance for j
                new_distance_j_left = instance.clustersDistanceMatrix[route[prev_customer_i]][route[j]];
                new_distance_j_right = instance.clustersDistanceMatrix[route[j]][route[next_customer_i]];
            }

            // Calculate new total distance
            double newDistance = this.solution.totalClusterRouteDistance - distance_i_left - distance_i_right - 
                distance_j_left - distance_j_right + new_distance_i_left + new_distance_i_right + 
                new_distance_j_left + new_distance_j_right;

            // If new distance is better
            if (newDistance + 0.01 < this.solution.totalClusterRouteDistance)
            {
                // Perform exchange
                int customer = route[i];
                route[i] = route[j];
                route[j] = customer;

                // If route is valid
                if (Functions.isValidClusterRoute(route))
                {
                    // Update distance
                    solution.totalClusterRouteDistance = newDistance;
                    return true;
                }
                else
                {
                    // Back to old route
                    route[j] = route[i];
                    route[i] = customer;
                }
            } // end if distance is better

            // Perform exchange is not possible
            return false;
        }
          
        // Try to swap all the clusters (one bye one) for all vehicles (one by one)
        public void swapVehicle(int[] clusterDemand)
        {
            // For each vehicle 1
            for (int vehicle1 = 0; vehicle1 < solution.clusterRouteForVehicule.Length; vehicle1++)
            {
                // For each vehicle 2
                for (int vehicle2 = 0; vehicle2 < solution.clusterRouteForVehicule.Length; vehicle2++)
                {
                    // When is not the same vehicle
                    if (vehicle1 != vehicle2)
                    {
                        // For each cluster 1 on vehicle 1
                        for (int cluster1 = 1; cluster1 < solution.clusterRouteForVehicule[vehicle1].Count; cluster1++)
                        {
                            // For each cluster 2 on vehicle 2
                            for (int cluster2 = 1; cluster2 < solution.clusterRouteForVehicule[vehicle2].Count; cluster2++)
                            {
                                // Calculate the space on vehicle if make a swap
                                int clusterSwappedV1 = solution.clusterRouteForVehicule[vehicle1][cluster1];
                                int clusterSwappedV2 = solution.clusterRouteForVehicule[vehicle2][cluster2];
                                int newSpaceV1 = solution.vehicleRemSpace[vehicle1] + clusterDemand[clusterSwappedV1] - clusterDemand[clusterSwappedV2];
                                int newSpaceV2 = solution.vehicleRemSpace[vehicle2] + clusterDemand[clusterSwappedV2] - clusterDemand[clusterSwappedV1];

                                // If swap is possible
                                if (newSpaceV1 > 0 && newSpaceV2 > 0 && clusterSwappedV1 != 0 && clusterSwappedV2 != 0 && clusterSwappedV1 != clusterSwappedV2)
                                {
                                    // Calculate old distances for each vehicle
                                    double oldDistanceVehicle1 = Functions.calculateClusterTravelDistanceForVehicle(solution.clusterRouteForVehicule[vehicle1], instance.clustersDistanceMatrix);
                                    double oldDistanceVehicle2 = Functions.calculateClusterTravelDistanceForVehicle(solution.clusterRouteForVehicule[vehicle2], instance.clustersDistanceMatrix);

                                    // Swap clusters
                                    solution.clusterRouteForVehicule[vehicle1][cluster1] = clusterSwappedV2;
                                    solution.clusterRouteForVehicule[vehicle2][cluster2] = clusterSwappedV1;

                                    // Calculate new distances for each vehicle
                                    double newDistanceVehicle1 = Functions.calculateClusterTravelDistanceForVehicle(solution.clusterRouteForVehicule[vehicle1], instance.clustersDistanceMatrix);
                                    double newDistanceVehicle2 = Functions.calculateClusterTravelDistanceForVehicle(solution.clusterRouteForVehicule[vehicle2], instance.clustersDistanceMatrix);

                                    // Calculate new total distance
                                    double newDistance = solution.totalClusterRouteDistance - (oldDistanceVehicle1 + oldDistanceVehicle2) + (newDistanceVehicle1 + newDistanceVehicle2);

                                    // If new distance is short
                                    if (newDistance < solution.totalClusterRouteDistance)
                                    {
                                        // Update distance and space remaining
                                        solution.totalClusterRouteDistance = newDistance;
                                        solution.vehicleRemSpace[vehicle1] = newSpaceV1;
                                        solution.vehicleRemSpace[vehicle2] = newSpaceV2;
                                    }
                                    // If new distance is not short
                                    else
                                    {
                                        // Undo swap
                                        solution.clusterRouteForVehicule[vehicle1][cluster1] = clusterSwappedV1;
                                        solution.clusterRouteForVehicule[vehicle2][cluster2] = clusterSwappedV2;
                                    }
                                } // End if swap is possible
                            } // End for cluster 2
                        } // End for cluster 1
                    } // End if is not the same vehicle
                } // End for vehicle 2
            } // End for vehicle 1

            //End
            return;
        }

        // Try to insert all the clusters (one bye one) on vehicle in all the others vehicles (one by one)
        public void insertVehicle(int[] clusterDemand)
        {
            // For each vehicle 1
            for (int vehicle1 = 0; vehicle1 < solution.clusterRouteForVehicule.Length; vehicle1++)
            {
                // For each vehicle 2
                for (int vehicle2 = 0; vehicle2 < solution.clusterRouteForVehicule.Length; vehicle2++)
                {
                    // If not the same vehicle
                    if (vehicle1 != vehicle2)
                    {
                        // For each cluster 1 on vehicle 1
                        for (int cluster1Idx = 1; cluster1Idx + 1 < solution.clusterRouteForVehicule[vehicle1].Count; cluster1Idx++)
                        {
                            // Select cluster
                            int clusterToInsert = solution.clusterRouteForVehicule[vehicle1][cluster1Idx];

                            // If swap is possible respect to the space remaining
                            if (solution.vehicleRemSpace[vehicle2] - clusterDemand[clusterToInsert] >= 0)
                            {
                                // Remove cluster 1 on vehicle 1
                                solution.clusterRouteForVehicule[vehicle1].Remove(clusterToInsert);

                                // Search the best index to insert cluster 1 on vehicle 2
                                Tuple<int, double> bestIndexAndDistance = bestIndexToInsertCluster(vehicle2, clusterToInsert);
                                int bestIndex = bestIndexAndDistance.Item1;
                                double newDistance = bestIndexAndDistance.Item2;

                                // If there is short distance
                                if (bestIndex != -1)
                                {
                                    // Insert cluster 1 on vehicle 2
                                    solution.clusterRouteForVehicule[vehicle2].Insert(bestIndex, clusterToInsert);

                                    // Update space remaining
                                    solution.vehicleRemSpace[vehicle1] += clusterDemand[clusterToInsert];
                                    solution.vehicleRemSpace[vehicle2] -= clusterDemand[clusterToInsert];

                                    // Update new distance
                                    solution.totalClusterRouteDistance = newDistance;
                                }
                                // If distance is not improved
                                else
                                {
                                    // Put back the cluster 1 on vehicle 1
                                    solution.clusterRouteForVehicule[vehicle1].Insert(cluster1Idx, clusterToInsert);
                                }
                            } // End if insert is possible
                        } // End for cluster 1 on vehicle 1
                    } // End for not same vehicle
                } // End for vehicle 2
            } // End for vehicle 1

            // End
            return;
        }
        
        // Return the best index to insert a cluster on a vehicle if improve solution distance
        private Tuple<int, double> bestIndexToInsertCluster(int vehicle, int clusterToInsert)
        {
            // Init variables
            int bestIndex = -1;
            int pathSize = solution.clusterRouteForVehicule[vehicle].Count;

            // Get solution acual best distance
            double totalClusterRouteDistance = solution.totalClusterRouteDistance;

            // For each cluster position on the vehicle route
            for (int clusterIdx = 1; clusterIdx + 1 < pathSize; clusterIdx++)
            {
                // Insert cluster on the clusterIdx-position
                solution.clusterRouteForVehicule[vehicle].Insert(clusterIdx, clusterToInsert);

                // Calculate new distance
                double newDistance = Functions.calculateTotalClusterTravelDistance(solution.clusterRouteForVehicule, instance.clustersDistanceMatrix);

                // If new distance is better
                if(newDistance + 0.01 < totalClusterRouteDistance)
                {
                    // Update best index and distance
                    bestIndex = clusterIdx;
                    totalClusterRouteDistance = newDistance;
                }

                // Remove cluster on the path 
                solution.clusterRouteForVehicule[vehicle].Remove(clusterToInsert);
            }
          
            // Return result
            return new Tuple<int, double>(bestIndex, totalClusterRouteDistance);
        }

        /*
         * 
         * Swap cluster beetween vehicles with random criteria
         * 1 - Select source vehicle_1 by random
         * 2 - Select destiny vehicle_2 by random
         * 3 - Select cluster_1 on vehicle_1 by random
         * 4 - Select cluster_2 on vehicle_2 by random
         * 5 - Try to swap cluster_1 and cluster_2 if is possible and better
         * 
         */
        public void interVehicleRandomSwap()
        {
            // Main cycle
            int iterator = 0;
            while (iterator < maxIterationsWithoutImprovementIVRS)
            {
                // Make random selections
                Random rnd = new Random();
                int numberOfVehicles = solution.clusterRouteForVehicule.Length;
                int vehicle1 = rnd.Next(0, numberOfVehicles);
                int vehicle2 = rnd.Next(0, numberOfVehicles);
                int clusterV1 = Functions.selectRandomElement(solution.clusterRouteForVehicule[vehicle1]);
                int clusterV2 = Functions.selectRandomElement(solution.clusterRouteForVehicule[vehicle2]);

                // Calculate remaining space
                int remSpaceV1 = solution.vehicleRemSpace[vehicle1] + instance.clusters_demand[clusterV1] - instance.clusters_demand[clusterV2];
                int remSpaceV2 = solution.vehicleRemSpace[vehicle2] + instance.clusters_demand[clusterV2] - instance.clusters_demand[clusterV1];

                // Check if swap is possible
                if (clusterV1 != 0 && clusterV2 != 0 && vehicle1 != vehicle2 && remSpaceV1 > 0 && remSpaceV2 > 0)
                {

                    //  Swap clusters
                    int idxClusterV1 = solution.clusterRouteForVehicule[vehicle1].IndexOf(clusterV1);
                    int idxClusterV2 = solution.clusterRouteForVehicule[vehicle2].IndexOf(clusterV2);

                    // Calculate old distances for each vehicle
                    double oldDistanceVehicle1 = Functions.calculateClusterTravelDistanceForVehicle(solution.clusterRouteForVehicule[vehicle1], instance.clustersDistanceMatrix);
                    double oldDistanceVehicle2 = Functions.calculateClusterTravelDistanceForVehicle(solution.clusterRouteForVehicule[vehicle2], instance.clustersDistanceMatrix);

                    // Swap clusters
                    solution.clusterRouteForVehicule[vehicle1][idxClusterV1] = clusterV2;
                    solution.clusterRouteForVehicule[vehicle2][idxClusterV2] = clusterV1;

                    // Calculate new distances for each vehicle
                    double newDistanceVehicle1 = Functions.calculateClusterTravelDistanceForVehicle(solution.clusterRouteForVehicule[vehicle1], instance.clustersDistanceMatrix);
                    double newDistanceVehicle2 = Functions.calculateClusterTravelDistanceForVehicle(solution.clusterRouteForVehicule[vehicle2], instance.clustersDistanceMatrix);

                    // Calculate new total distance
                    double newDistance = solution.totalClusterRouteDistance - (oldDistanceVehicle1 + oldDistanceVehicle2) + (newDistanceVehicle1 + newDistanceVehicle2);

                    // Verify is new distance is short 
                    if (newDistance + 0.01 < solution.totalClusterRouteDistance)
                    {
                        // Update new vehicle space remaining
                        solution.vehicleRemSpace[vehicle1] += instance.clusters_demand[clusterV1];
                        solution.vehicleRemSpace[vehicle1] -= instance.clusters_demand[clusterV2];
                        solution.vehicleRemSpace[vehicle2] += instance.clusters_demand[clusterV2];
                        solution.vehicleRemSpace[vehicle2] -= instance.clusters_demand[clusterV1];

                        // Update new distance
                        solution.totalClusterRouteDistance = newDistance;

                        // Reset iterator
                        iterator = 0;
                    }
                    // If is not better back the swap
                    else
                    {
                        solution.clusterRouteForVehicule[vehicle1][idxClusterV1] = clusterV1;
                        solution.clusterRouteForVehicule[vehicle2][idxClusterV2] = clusterV2;
                        iterator++;
                    }
                }
                // If swap is not possible 
                else
                {
                    // Increase iterator
                    iterator++;
                }
            }

            // End
            return;
        }

        /*
         * 
         * Change the vehicle of a cluster into other vehicle 
         * 1 - Select source vehicle_1 by random
         * 2 - Select destiny vehicle_2 by random
         * 3 - Select cluster_1 on vehicle_1 by random
         * 4 - Try to insert cluster_1 and in vehicle_2 if is possible
         * 
         */
        public void interVehicleRandomChange(int[] clusterDemand)
        {
            // Main Cycle
            int iterator = 0;
            while (iterator < maxIterationsWithoutImprovementIVRC)
            {
                // Make random selections
                Random rnd = new Random();
                int numberOfVehicles = solution.clusterRouteForVehicule.Length;
                int vehicle1 = rnd.Next(0, numberOfVehicles);
                int vehicle2 = rnd.Next(0, numberOfVehicles);
                int clusterV1 = Functions.selectRandomElement(solution.clusterRouteForVehicule[vehicle1]);

                // Set improve to false
                bool improve = false;

                // Verify if insert is possible
                if (clusterV1 != 0 && vehicle1 != vehicle2 && solution.vehicleRemSpace[vehicle2] - clusterDemand[clusterV1] >= 0)
                {
                    // Calculate old distances for each vehicle
                    double oldDistanceVehicle1 = Functions.calculateClusterTravelDistanceForVehicle(solution.clusterRouteForVehicule[vehicle1], instance.clustersDistanceMatrix);
                    double oldDistanceVehicle2 = Functions.calculateClusterTravelDistanceForVehicle(solution.clusterRouteForVehicule[vehicle2], instance.clustersDistanceMatrix);

                    // Save index of cluster for re-insert in case of not improve
                    int idxClusterV1 = solution.clusterRouteForVehicule[vehicle1].IndexOf(clusterV1);

                    // Remone cluster from vehicle 1
                    solution.clusterRouteForVehicule[vehicle1].Remove(clusterV1);

                    // Calculate new distances for vehicle 1
                    double newDistanceVehicle1 = Functions.calculateClusterTravelDistanceForVehicle(solution.clusterRouteForVehicule[vehicle1], instance.clustersDistanceMatrix);
     
                    // Seach the best position to insert the cluster on the vehicle 2
                    for (int i = 0; i + 1 < solution.clusterRouteForVehicule[vehicle2].Count; i++)
                    {
                        // Insert cluster on vehicle 2
                        solution.clusterRouteForVehicule[vehicle2].Insert(i + 1, clusterV1);

                        // Calculate new for vehicle 2
                        double newDistanceVehicle2 = Functions.calculateClusterTravelDistanceForVehicle(solution.clusterRouteForVehicule[vehicle2], instance.clustersDistanceMatrix);

                        // Calculate new total distance
                        double newDistance = solution.totalClusterRouteDistance - (oldDistanceVehicle1 + oldDistanceVehicle2) + (newDistanceVehicle1 + newDistanceVehicle2);

                        // If distance is short
                        if (newDistance + 0.01 < solution.totalClusterRouteDistance)
                        {
                            // Update new distance and the space remaining on vehicles
                            solution.totalClusterRouteDistance = newDistance;
                            solution.vehicleRemSpace[vehicle1] += clusterDemand[clusterV1];
                            solution.vehicleRemSpace[vehicle2] -= clusterDemand[clusterV1];

                            // Reset iterator
                            iterator = 0;

                            // Set improve and break
                            improve = true;
                            break;
                        }
                        // If distance is not better
                        else
                        {
                            // Remove cluster from i-position on vehicle 2
                            solution.clusterRouteForVehicule[vehicle2].Remove(clusterV1);
                        }
                    }
                    // If not improve reached 
                    if (!improve)
                    {
                        // Back the cluster to the vehicle 1
                        solution.clusterRouteForVehicule[vehicle1].Insert(idxClusterV1, clusterV1);
                        improve = false;
                    }
                }

                // Increase iterator
                iterator++;
            }

            // End
            return;
        }
          
    }
}