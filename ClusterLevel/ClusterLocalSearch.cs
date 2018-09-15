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
        public int maxIterationsWithoutImprovementIVRS { get; set; }
        public int maxIterationsWithoutImprovementIVRC { get; set; }
        public int maxIterationsWithoutImprovementIV { get; set; }
        public int maxIterationsWithoutImprovementSV { get; set; }
        public int maxIterationsWithoutImprovementSC { get; set; }
        public CluVRPSolution solution { get; set; }
        public CluVRPInstance instance;

        // Constructor
        public ClusterLocalSearch(CluVRPSolution solution,
            CluVRPInstance instance,
            int maxIterationsWithoutImprovementTwoOpt = 100,
            int maxIterationsWithoutImprovementRelocate = 100,
            int maxIterationsWithoutImprovementExchange = 100,
            int maxIterationsWithoutImprovementIVRS = 100,
            int maxIterationsWithoutImprovementIVRC = 100,
            int maxIterationsWithoutImprovementIV = 100,
            int maxIterationsWithoutImprovementSV = 100,
            int maxIterationsWithoutImprovementSC = 100
            )
        {
            this.solution = solution;
            this.instance = instance;
            this.maxIterationsWithoutImprovementTwoOpt = maxIterationsWithoutImprovementTwoOpt;
            this.maxIterationsWithoutImprovementRelocate = maxIterationsWithoutImprovementRelocate;
            this.maxIterationsWithoutImprovementExchange = maxIterationsWithoutImprovementExchange;
            this.maxIterationsWithoutImprovementIVRS = maxIterationsWithoutImprovementIVRS;
            this.maxIterationsWithoutImprovementIVRC = maxIterationsWithoutImprovementIVRC;
            this.maxIterationsWithoutImprovementIV = maxIterationsWithoutImprovementIV;
            this.maxIterationsWithoutImprovementSV = maxIterationsWithoutImprovementSV;
            this.maxIterationsWithoutImprovementSC = maxIterationsWithoutImprovementSC;
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
                            if (newDistance + 0.000001 < bestDistance[vehicle] && Functions.isValidClusterRoute(newRoute))
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
            if (newDistance + 0.000001 < solution.totalClusterRouteDistance)
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
            if (newDistance + 0.000001 < this.solution.totalClusterRouteDistance)
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

        // Swap cluster the same vehicle
        public void SwapClusters()
        {
            // For each vehicle
            for(int vehicle = 0; vehicle < solution.clusterRouteForVehicule.Length; vehicle++)
            {
                // To swap cluster need at least 4 clusters
                if (solution.clusterRouteForVehicule[vehicle].Count < 4) continue;

                // Main cycle
                int iterator = 0;
                while (iterator < maxIterationsWithoutImprovementSC)
                {
                    // For each cluster1
                    for (int clusterIt1 = 1; clusterIt1 + 1 < solution.clusterRouteForVehicule[vehicle].Count; clusterIt1++)
                    {
                        // For each next or last to claster 1
                        for (int clusterIt2 = 1; clusterIt2 + 1 < solution.clusterRouteForVehicule[vehicle].Count; clusterIt2++)
                        {
                            if (clusterIt1 != clusterIt2)
                            {
                                int lastCluster1 = solution.clusterRouteForVehicule[vehicle][clusterIt1 - 1];
                                int cluster1 = solution.clusterRouteForVehicule[vehicle][clusterIt1];
                                int nextCluster1 = solution.clusterRouteForVehicule[vehicle][clusterIt1 + 1];

                                int lastCluster2 = solution.clusterRouteForVehicule[vehicle][clusterIt2 - 1];
                                int cluster2 = solution.clusterRouteForVehicule[vehicle][clusterIt2];
                                int nextCluster2 = solution.clusterRouteForVehicule[vehicle][clusterIt2 + 1];

                                /*
                                double oldDistance1 = instance.clustersDistanceMatrix[lastCluster1][cluster1] +
                                    instance.clustersDistanceMatrix[cluster1][nextCluster1];
                                double oldDistance2 = instance.clustersDistanceMatrix[lastCluster2][cluster2] +
                                    instance.clustersDistanceMatrix[cluster2][nextCluster2];
                                double newDistance1 = instance.clustersDistanceMatrix[lastCluster1][cluster2] +
                                    instance.clustersDistanceMatrix[cluster2][nextCluster1];
                                double newDistance2 = instance.clustersDistanceMatrix[lastCluster2][cluster1] +
                                    instance.clustersDistanceMatrix[cluster1][nextCluster2];

                                if (newDistance1 + newDistance2 < oldDistance1 + oldDistance2)
                                {
                                    solution.clusterRouteForVehicule[vehicle][clusterIt1] = cluster2;
                                    solution.clusterRouteForVehicule[vehicle][clusterIt2] = cluster1;
                                }*/
                                double oldDistance = Functions.calculateClusterTravelDistanceForVehicle(solution.clusterRouteForVehicule[vehicle], instance.clustersDistanceMatrix);
                                solution.clusterRouteForVehicule[vehicle][clusterIt1] = cluster2;
                                solution.clusterRouteForVehicule[vehicle][clusterIt2] = cluster1;
                                double newDistance = Functions.calculateClusterTravelDistanceForVehicle(solution.clusterRouteForVehicule[vehicle], instance.clustersDistanceMatrix);

                                // If new distance is not better
                                if (oldDistance < newDistance + 0.00001)
                                {
                                    solution.clusterRouteForVehicule[vehicle][clusterIt1] = cluster1;
                                    solution.clusterRouteForVehicule[vehicle][clusterIt2] = cluster2;
                                    iterator++;
                                }
                                // Else new distance is better
                                else
                                {
                                    // Reset iterator
                                    iterator = 0;
                                }

                            }
                            else
                            {
                                // Increase iterator
                                iterator++;
                            } // End cluster1 != cluster2
                        } // End each cluster2
                    }// End each cluster1
                } // End main cycle
            } // End each vehicle

            // End function
            return;
        }
          
        // Try to swap all the clusters (one bye one) for all vehicles (one by one)
        public void swapVehicle(int[] clusterDemand)
        {
            // More than 1 vehicle is needed
            if (solution.clusterRouteForVehicule.Length < 2) return;

            // Main cycle
            int iterations = 0;
            while (iterations < maxIterationsWithoutImprovementSV)
            {
                // For random order on iterations
                List<int> rndPosition = new List<int>();
                for (int i = 0; i < solution.clusterRouteForVehicule.Length; i++) rndPosition.Add(i);
                Functions.Shuffle<int>(new Random(), rndPosition);

                // For each vehicle 1
                for (int vehicleIt1 = 0; vehicleIt1 < solution.clusterRouteForVehicule.Length; vehicleIt1++)
                {
                    // For each vehicle 2
                    for (int vehicleIt2 = 0; vehicleIt2 < solution.clusterRouteForVehicule.Length; vehicleIt2++)
                    {
                        // When is not the same vehicle
                        if (vehicleIt1 != vehicleIt2)
                        {
                            // Select vehicle index from random position
                            int vehicle1 = rndPosition[vehicleIt1];
                            int vehicle2 = rndPosition[vehicleIt2];

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

                                            // Reset iterator
                                            iterations = 0;

                                        }
                                        // If new distance is not short
                                        else
                                        {
                                            // Undo swap
                                            solution.clusterRouteForVehicule[vehicle1][cluster1] = clusterSwappedV1;
                                            solution.clusterRouteForVehicule[vehicle2][cluster2] = clusterSwappedV2;

                                            // Increase iterator
                                            iterations++;
                                        }
                                    }
                                    else
                                    {
                                        iterations++;
                                    } 
                                        // End if swap is possible
                                } // End for cluster 2
                            } // End for cluster 1
                        } // End if is not the same vehicle
                    } // End for vehicle 2
                } // End for vehicle 1
            }

            //End
            return;
        }

        // Try to insert all the clusters (one bye one) on vehicle in all the others vehicles (one by one)
        public void insertVehicle(int[] clusterDemand)
        {
            // More than 1 vehicle is needed
            if (solution.clusterRouteForVehicule.Length < 2) return;

            // Start main iterations
            int iterations = 0;
            while (iterations < maxIterationsWithoutImprovementIV)
            {
                // For random order on iterations
                List<int> rndPosition = new List<int>();
                for (int i = 0; i < solution.clusterRouteForVehicule.Length; i++) rndPosition.Add(i);
                Functions.Shuffle<int>(new Random(), rndPosition);

                // For each vehicle 1
                for (int vehicleIt1 = 0; vehicleIt1 < solution.clusterRouteForVehicule.Length; vehicleIt1++)
                {
                    // For each vehicle 2
                    for (int vehicleIt2 = 0; vehicleIt2 < solution.clusterRouteForVehicule.Length; vehicleIt2++)
                    {
                        // Select vehicle index from random position
                        int vehicle1 = rndPosition[vehicleIt1];
                        int vehicle2 = rndPosition[vehicleIt2];

                        // No empty vehicles for barreta instances
                        bool notEmptyVehicle = !(solution.clusterRouteForVehicule[vehicle1].Count <= 3 && (instance.instance_type == Instance.GoldenBattarra || instance.instance_type == Instance.GVRP));

                        // If not the same vehicle
                        if (vehicle1 != vehicle2 && notEmptyVehicle)
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

                                        // Reset iterator
                                        iterations = 0;
                                    }
                                    // If distance is not improved
                                    else
                                    {
                                        // Put back the cluster 1 on vehicle 1
                                        solution.clusterRouteForVehicule[vehicle1].Insert(cluster1Idx, clusterToInsert);

                                        // Increase iterator
                                        iterations++;
                                    }
                                }else
                                {
                                    iterations++;
                                }
                                // End if insert is possible
                            } // End for cluster 1 on vehicle 1
                        } // End for not same vehicle
                    } // End for vehicle 2
                } // End for vehicle 1
            }

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
                if(newDistance + 0.000001 < totalClusterRouteDistance)
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
                    if (newDistance + 0.000001 < solution.totalClusterRouteDistance)
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

                // No empty vehicles for barreta instances
                bool notEmptyVehicle = !(solution.clusterRouteForVehicule[vehicle1].Count > 2 && (instance.instance_type == Instance.GoldenBattarra || instance.instance_type == Instance.GVRP));

                // Verify if insert is possible
                if (notEmptyVehicle && clusterV1 != 0 && vehicle1 != vehicle2 && solution.vehicleRemSpace[vehicle2] - clusterDemand[clusterV1] >= 0)
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
                        if (newDistance + 0.000001 < solution.totalClusterRouteDistance)
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
