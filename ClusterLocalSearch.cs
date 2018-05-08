using System;
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
        public CluVRPSolution solution { get; set; }
        public CluVRPInstance instance;

        // Constructor
        public ClusterLocalSearch(CluVRPSolution solution,
            CluVRPInstance instance,
            int maxIterationsWithoutImprovement = 100,
            int maxIterationsWithoutImprovementRelocate = 100,
            int maxIterationsWithoutImprovementExchange = 100)
        {
            this.solution = solution;
            this.instance = instance;
            this.maxIterationsWithoutImprovementTwoOpt = maxIterationsWithoutImprovement;
            this.maxIterationsWithoutImprovementRelocate = maxIterationsWithoutImprovementRelocate;
            this.maxIterationsWithoutImprovementExchange = maxIterationsWithoutImprovementExchange;
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
                bestDistance[vehicle] = ClusterGRASP.calculateClusterTravelDistanceForVehicle(route, instance.clustersDistanceMatrix);

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
                            double newDistance = ClusterGRASP.calculateClusterTravelDistanceForVehicle(newRoute, instance.clustersDistanceMatrix);

                            // If distance if better and route is valid
                            if (newDistance + 0.5 < bestDistance[vehicle] && isValidClusterRoute(newRoute))
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
            if (newDistance + 0.5 < solution.totalClusterRouteDistance)
            {
                // Perform realocate
                int customer = route[i];
                route.RemoveAt(i);
                route.Insert(j, customer);

                // If route is valid
                if (isValidClusterRoute(route))
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
            // To be on the way
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
            if (newDistance + 0.5 < this.solution.totalClusterRouteDistance)
            {
                // Perform exchange
                int customer = route[i];
                route[i] = route[j];
                route[j] = customer;

                // If route is valid
                if (isValidClusterRoute(route))
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
                    solution.clusterRouteForVehicule[vehicle] = route;
                }
            } // end if distance is better

            // Perform exchange is not possible
            return false;
        }

        public void interVehicleRandomSwap()
        {
            int iterator = 0;
            while (iterator < maxIterationsWithoutImprovementTwoOpt)
            {
                Random rnd = new Random();
                int numberOfVehicles = solution.clusterRouteForVehicule.Length;
                int vehicle1 = rnd.Next(0, numberOfVehicles);
                int vehicle2 = rnd.Next(0, numberOfVehicles);
                int clusterV1 = Functions.selectRandomElement(solution.clusterRouteForVehicule[vehicle1]);
                int clusterV2 = Functions.selectRandomElement(solution.clusterRouteForVehicule[vehicle2]);
                int remSpaceV1 = solution.vehicleRemSpace[vehicle1] + instance.clusters_demand[clusterV1] - instance.clusters_demand[clusterV2];
                int remSpaceV2 = solution.vehicleRemSpace[vehicle2] + instance.clusters_demand[clusterV2] - instance.clusters_demand[clusterV1];

                if (clusterV1 != 0 && clusterV2 != 0 && vehicle1 != vehicle2 && remSpaceV1 > 0 && remSpaceV2 > 0)
                {
                    int idxClusterV1 = solution.clusterRouteForVehicule[vehicle1].IndexOf(clusterV1);
                    int idxClusterV2 = solution.clusterRouteForVehicule[vehicle2].IndexOf(clusterV2);
                    solution.clusterRouteForVehicule[vehicle1][idxClusterV1] = clusterV2;
                    solution.clusterRouteForVehicule[vehicle2][idxClusterV2] = clusterV1;
                    double newDistance = ClusterGRASP.calculateTotalClusterTravelDistance(solution.clusterRouteForVehicule, instance.clustersDistanceMatrix);
                    if (newDistance + 0.5 < solution.totalClusterRouteDistance)
                    {
                        solution.vehicleRemSpace[vehicle1] += instance.clusters_demand[clusterV1];
                        solution.vehicleRemSpace[vehicle1] -= instance.clusters_demand[clusterV2];
                        solution.vehicleRemSpace[vehicle2] += instance.clusters_demand[clusterV2];
                        solution.vehicleRemSpace[vehicle2] -= instance.clusters_demand[clusterV1];
                        solution.totalClusterRouteDistance = newDistance;
                        iterator = 0;
                    }
                    else
                    {
                        solution.clusterRouteForVehicule[vehicle1][idxClusterV1] = clusterV1;
                        solution.clusterRouteForVehicule[vehicle2][idxClusterV2] = clusterV2;
                        iterator++;
                    }
                }
                else
                {
                    iterator++;
                }
            }
            return;
        }

        public void interVehicleRandomInsert(int[] clusterDemand)
        {
            int iterator = 0;
            while (iterator < maxIterationsWithoutImprovementExchange)
            {
                Random rnd = new Random();
                int numberOfVehicles = solution.clusterRouteForVehicule.Length;
                int vehicle1 = rnd.Next(0, numberOfVehicles);
                int vehicle2 = rnd.Next(0, numberOfVehicles);
                int clusterV1 = Functions.selectRandomElement(solution.clusterRouteForVehicule[vehicle1]);
                bool improve = false;
                if (clusterV1 !=0 && vehicle1 != vehicle2 && solution.vehicleRemSpace[vehicle2] - clusterDemand[clusterV1] >= 0)
                {
                    int idxClusterV1 = solution.clusterRouteForVehicule[vehicle1].IndexOf(clusterV1);
                    solution.clusterRouteForVehicule[vehicle1].Remove(clusterV1);
                    for (int i = 0; i < solution.clusterRouteForVehicule[vehicle2].Count; i++)
                    {
                        solution.clusterRouteForVehicule[vehicle2].Insert(i + 1, clusterV1);
                        double newDistance = ClusterGRASP.calculateTotalClusterTravelDistance(solution.clusterRouteForVehicule, instance.clustersDistanceMatrix);
                        if (newDistance < solution.totalClusterRouteDistance)
                        {
                            solution.totalClusterRouteDistance = newDistance;
                            solution.vehicleRemSpace[vehicle1] += clusterDemand[clusterV1];
                            solution.vehicleRemSpace[vehicle2] -= clusterDemand[clusterV1];
                            iterator = 0;
                            improve = true;
                            break;
                        }
                        else
                        {
                            solution.clusterRouteForVehicule[vehicle2].Remove(clusterV1);
                        }
                    }
                    if (!improve)
                    {
                        solution.clusterRouteForVehicule[vehicle1].Insert(idxClusterV1, clusterV1);
                        improve = false;
                    }                    
                }
                iterator++;
            }
            return;
        }

        public void swapVehicle(int[] clusterDemand)
        {
            for (int vehicle1 = 0; vehicle1 < solution.clusterRouteForVehicule.Length; vehicle1++)
            {
                for (int vehicle2 = 0; vehicle2 < solution.clusterRouteForVehicule.Length; vehicle2++)
                {
                    if (vehicle1 != vehicle2)
                    {
                        for (int cluster1 = 1; cluster1 < solution.clusterRouteForVehicule[vehicle1].Count; cluster1++)
                        {
                            for (int cluster2 = 1; cluster2 < solution.clusterRouteForVehicule[vehicle2].Count; cluster2++)
                            {
                                int clusterSwappedV1 = solution.clusterRouteForVehicule[vehicle1][cluster1];
                                int clusterSwappedV2 = solution.clusterRouteForVehicule[vehicle2][cluster2];
                                int newSpaceV1 = solution.vehicleRemSpace[vehicle1] + clusterDemand[clusterSwappedV1] - clusterDemand[clusterSwappedV2];
                                int newSpaceV2 = solution.vehicleRemSpace[vehicle2] + clusterDemand[clusterSwappedV2] - clusterDemand[clusterSwappedV1];

                                if (newSpaceV1 > 0 && newSpaceV2 > 0 && clusterSwappedV1 != 0 && clusterSwappedV2 != 0 && clusterSwappedV1 != clusterSwappedV2)
                                {
                                    solution.clusterRouteForVehicule[vehicle1][cluster1] = clusterSwappedV2;
                                    solution.clusterRouteForVehicule[vehicle2][cluster2] = clusterSwappedV1;
                                    double newDistance = ClusterGRASP.calculateTotalClusterTravelDistance(solution.clusterRouteForVehicule, instance.clustersDistanceMatrix);

                                    if (newDistance < solution.totalClusterRouteDistance)
                                    {
                                        solution.totalClusterRouteDistance = newDistance;
                                        solution.vehicleRemSpace[vehicle1] = newSpaceV1;
                                        solution.vehicleRemSpace[vehicle2] = newSpaceV2;
                                    }
                                    else
                                    {
                                        solution.clusterRouteForVehicule[vehicle1][cluster1] = clusterSwappedV1;
                                        solution.clusterRouteForVehicule[vehicle2][cluster2] = clusterSwappedV2;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        public void insertVehicle(int[] clusterDemand)
        {

            for (int vehicle1 = 0; vehicle1 < solution.clusterRouteForVehicule.Length; vehicle1++)
            {
                for (int vehicle2 = 0; vehicle2 < solution.clusterRouteForVehicule.Length; vehicle2++)
                {
                    if (vehicle1 != vehicle2)

                        for (int cluster1Idx = 1; cluster1Idx + 1 < solution.clusterRouteForVehicule[vehicle1].Count; cluster1Idx++)
                        {
                            int clusterToInsert = solution.clusterRouteForVehicule[vehicle1][cluster1Idx];

                            if (solution.vehicleRemSpace[vehicle2] - clusterDemand[clusterToInsert] >= 0)
                            {
                                solution.clusterRouteForVehicule[vehicle1].Remove(clusterToInsert);
                                int bestIndex = bestIndexToInsertCluster(vehicle2, clusterToInsert);
                                if (bestIndex != -1)
                                {
                                    solution.clusterRouteForVehicule[vehicle2].Insert(bestIndex, clusterToInsert);
                                    solution.vehicleRemSpace[vehicle1] += clusterDemand[clusterToInsert];
                                    solution.vehicleRemSpace[vehicle2] -= clusterDemand[clusterToInsert];

                                }
                                else
                                {
                                    //solution.clusterRouteForVehicule[vehicle2].Remove(clusterToInsert);
                                    solution.clusterRouteForVehicule[vehicle1].Insert(cluster1Idx, clusterToInsert);
                                }
                            }
                        }
                    }
                }
            }
        
        private int bestIndexToInsertCluster(int vehicle, int clusterToInsert)
        {
            int bestIndex = -1;
            int pathSize = solution.clusterRouteForVehicule[vehicle].Count;
            for (int clusterIdx = 1; clusterIdx + 1 < pathSize; clusterIdx++)
            {
                solution.clusterRouteForVehicule[vehicle].Insert(clusterIdx, clusterToInsert);
                double newDistance = ClusterGRASP.calculateTotalClusterTravelDistance(solution.clusterRouteForVehicule, instance.clustersDistanceMatrix);

                if(newDistance < solution.totalClusterRouteDistance)
                {
                    bestIndex = clusterIdx;
                    solution.totalClusterRouteDistance = newDistance;
                }
                solution.clusterRouteForVehicule[vehicle].Remove(clusterToInsert);
            }
          

            return bestIndex;
        }

        // Verify is cluster route is valid (start and end in depot)
        public bool isValidClusterRoute(List<int> route)
        {
            if (route.Count > 0)
            {
                return (route[0] == 0 && route[route.Count - 1] == 0);
            }
            return false;
        }
    }
}
