using cluvrp_grasp.CluVRP;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace cluvrp_grasp
{
    enum CluVRPVersion { Strong, Weak, None }

    class CluVRPGrasp
    {

        // Main GRASP handle main iteration depending if use Complete version or TwoPhase
        public static CluVRPSolution Grasp(CluVRPInstance instance, Parameters parameters)
        {
            // For best solution
            CluVRPSolution bestSolution = new CluVRPSolution(instance);

            // Main cycle
            int iterator = 0;
            while (iterator < parameters.CluVRP_GRASPIterations)
            {
                // New Grasp for Cluster level instance
                ClusterGRASP clusterGrasp = new ClusterGRASP(instance, parameters);

                // Execute Grasp procedure
                var totalWatch = System.Diagnostics.Stopwatch.StartNew();
                CluVRPSolution cluVRPSolution = clusterGrasp.Grasp();
                cluVRPSolution.clusterLevelTime = totalWatch.ElapsedMilliseconds;

                // If solutions is not available continue with next iteration
                if (cluVRPSolution.clusterRouteForVehicule == null)
                {
                    iterator++;
                    continue;
                }

                // Verify if cluster solution is correct
                cluVRPSolution.verifyClusterSolution(instance);

                // New Grasp for Cluster level instance
                CustomerLevel customerGrasp = null;
                if (parameters.CluVRP_Version == CluVRPVersion.Strong)
                {
                    customerGrasp = new CustomerStrongGRASP(instance, cluVRPSolution, parameters);
                }
                else if (parameters.CluVRP_Version == CluVRPVersion.Weak)
                {
                    customerGrasp = new CustomerWeakGRASP(instance, cluVRPSolution, parameters);
                }

                // Execute Grasp procedure
                totalWatch = System.Diagnostics.Stopwatch.StartNew();
                customerGrasp.Grasp();
                cluVRPSolution.customerLevelTime = totalWatch.ElapsedMilliseconds;

                //if solution is not available continue with next iteration
                if (cluVRPSolution.customersPaths == null && cluVRPSolution.customersWeakRoute == null)
                {
                    continue;
                }

                // Perform LS
                if ((parameters.CluVRP_LS_SwapClusters> 0 || parameters.CluVRP_LS_SwapVehicle > 0) && parameters.CluVRP_Version == CluVRPVersion.Strong)
                {
                    cluVRPLocalSearchs(cluVRPSolution, instance, parameters);
                }

                // Verify if customer solution is correct
                if (parameters.CluVRP_Version == CluVRPVersion.Strong)
                {
                    cluVRPSolution.verifyCustomerSolution(instance);
                }
                else if (parameters.CluVRP_Version == CluVRPVersion.Weak)
                {
                    cluVRPSolution.verifyCustomerWeakSolution(instance);
                }

                // Update best solution
                if (cluVRPSolution.totalCustomerRouteDistance < bestSolution.totalCustomerRouteDistance)
                {
                    bestSolution = cluVRPSolution;
                    bestSolution.cluVRPIterations = iterator;
                }

                // Increase iterator
                iterator++;
            }

            // Return best solution
            return bestSolution;
        }
        
        // New LS to final solutions
        public static void cluVRPLocalSearchs(CluVRPSolution cluVRPSolution, CluVRPInstance instance, Parameters parameters) {

            // Perform LS at cluster level
            double oldTotalDistance = cluVRPSolution.totalCustomerRouteDistance;
            swapClusters(cluVRPSolution, instance, parameters);
            swapVehicle(cluVRPSolution, instance, parameters);

            // If some cluster position change make LS at customer level
            if (cluVRPSolution.totalCustomerRouteDistance < oldTotalDistance)
            {
                // Create a local search handler for cluster-level problem
                CustomerStrongLocalSearch customerLocalSearch = new CustomerStrongLocalSearch(cluVRPSolution,
                    instance,
                    parameters.Customer_LS_TwoOpt_Iterations,
                    parameters.Customer_LS_Relocate_Iterations,
                    parameters.Customer_LS_Exchange_Iterations,
                    parameters.Customer_LS_SwapCustomers
                    );

                // Perform one iteration of LS at customer level
                customerLocalSearch.twoOpt();
                customerLocalSearch.exchange();
                customerLocalSearch.relocate();
                customerLocalSearch.swapCustomers();
            }

            // End
            return;
        }

        // Swap Cluster heuristic (diff to the cluster levels because the customers paths has been builded)
        public static void swapClusters(CluVRPSolution solution, CluVRPInstance instance, Parameters parameters) {

            // Calculate old distances for each vehicle
            double[] bestVehicleDistance = new double[solution.clusterRouteForVehicule.Length];
            for (int vehicleIt = 0; vehicleIt < bestVehicleDistance.Length; vehicleIt++)
            {
                bestVehicleDistance[vehicleIt] = Functions.calculateTotalTravelDistance(solution.customersPaths, instance.customersDistanceMatrix, vehicleIt);
            }

            // For each vehicle
            for (int vehicle = 0; vehicle < solution.customersPaths.Length; vehicle++)
            {
                // Main cycle
                while (true)
                {
                    // If solution improves try with new iteration
                    bool solutionImproves = false;

                    // For each cluster1
                    for (int clusterIt1 = 1; clusterIt1 + 1 < solution.customersPaths[vehicle].Length; clusterIt1++)
                    {
                        // For each cluster2
                        for (int clusterIt2 = 1; clusterIt2 + 1 < solution.customersPaths[vehicle].Length; clusterIt2++)
                        {
                            // If is not the same cluster
                            if (clusterIt1 != clusterIt2)
                            {
                                // Calculate last and next cluster for cluster1
                                List<int> cluster1Last = solution.customersPaths[vehicle][clusterIt1 - 1];
                                List<int> cluster1 = solution.customersPaths[vehicle][clusterIt1];
                                List<int> cluster1Next = solution.customersPaths[vehicle][clusterIt1 + 1];

                                // Calculate last and next cluster for cluster2
                                List<int> cluster2Last = solution.customersPaths[vehicle][clusterIt2 - 1];
                                List<int> cluster2 = solution.customersPaths[vehicle][clusterIt2];
                                List<int> cluster2Next = solution.customersPaths[vehicle][clusterIt2 + 1];

                                // Perform swap
                                solution.customersPaths[vehicle][clusterIt1] = cluster2;
                                solution.customersPaths[vehicle][clusterIt2] = cluster1;
                                Functions.Swap(solution.clusterRouteForVehicule[vehicle], clusterIt1, clusterIt2);

                                // Calculate new distance
                                double newDistance = Functions.calculateTotalTravelDistance(solution.customersPaths, instance.customersDistanceMatrix, vehicle);

                                // If new distance is not better
                                if (solution.vehiculeRouteDistance[vehicle] <= newDistance + 0.00001)
                                {
                                    // Back the changes
                                    solution.customersPaths[vehicle][clusterIt1] = cluster1;
                                    solution.customersPaths[vehicle][clusterIt2] = cluster2;
                                    Functions.Swap(solution.clusterRouteForVehicule[vehicle], clusterIt1, clusterIt2);
                                }
                                else
                                {
                                    // If new solution is better update distance
                                    solution.vehiculeRouteDistance[vehicle] = newDistance;
                                    solutionImproves = true;
                                }
                            }
                        }
                    }

                    // If solution not improves jump to next vehicle
                    if (!solutionImproves) break;
                }
            }

            // End
            return;
        }

        // Try to swap all the clusters (one bye one) for all vehicles (one by one)
        public static void swapVehicle(CluVRPSolution solution, CluVRPInstance instance, Parameters parameters)
        {
            // Cluster demand 
            int[] clusterDemand = instance.clusters_demand;

            // More than 1 vehicle is needed
            if (solution.clusterRouteForVehicule.Length < 2) return;

            // Main cycle
            int iterations = 0;
            while (iterations < parameters.CluVRP_LS_SwapVehicle)
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
                                        int lastClusterSwappedV1 = solution.clusterRouteForVehicule[vehicle1][cluster1 - 1];
                                        int lastClusterSwappedV2 = solution.clusterRouteForVehicule[vehicle2][cluster2 - 1];
                                        int nextClusterSwappedV1 = solution.clusterRouteForVehicule[vehicle1][cluster1 + 1];
                                        int nextClusterSwappedV2 = solution.clusterRouteForVehicule[vehicle2][cluster2 + 1];

                                        // Calculate old distances for each vehicle
                                        double oldDistanceVehicle1 = Functions.calculateCustomerTotalTravelDistanceForVehicle(solution.customersPaths[vehicle1], instance.customersDistanceMatrix, instance);
                                        double oldDistanceVehicle2 = Functions.calculateCustomerTotalTravelDistanceForVehicle(solution.customersPaths[vehicle2], instance.customersDistanceMatrix, instance);

                                        // Swap clusters
                                        List<int> cluster1Swp = solution.customersPaths[vehicle1][cluster1];
                                        List<int> cluster2Swp = solution.customersPaths[vehicle2][cluster2];
                                        solution.customersPaths[vehicle1][cluster1] = cluster2Swp;
                                        solution.customersPaths[vehicle2][cluster2] = cluster1Swp;

                                        // Calculate new distances for each vehicle
                                        double newDistanceVehicle1 = Functions.calculateCustomerTotalTravelDistanceForVehicle(solution.customersPaths[vehicle1], instance.customersDistanceMatrix, instance);
                                        double newDistanceVehicle2 = Functions.calculateCustomerTotalTravelDistanceForVehicle(solution.customersPaths[vehicle2], instance.customersDistanceMatrix, instance);

                                        // Calculate new total distance
                                        double newDistance = solution.totalClusterRouteDistance - (oldDistanceVehicle1 + oldDistanceVehicle2) + (newDistanceVehicle1 + newDistanceVehicle2);

                                        // If new distance is short
                                        if (newDistance + 0.0001 < solution.totalClusterRouteDistance)
                                        {
                                            // Update distance and space remaining
                                            solution.totalClusterRouteDistance = newDistance;
                                            solution.vehicleRemSpace[vehicle1] = newSpaceV1;
                                            solution.vehicleRemSpace[vehicle2] = newSpaceV2;
                                            solution.clusterRouteForVehicule[vehicle1][cluster1] = clusterSwappedV2;
                                            solution.clusterRouteForVehicule[vehicle2][cluster1] = clusterSwappedV1;

                                            // Reset iterator
                                            iterations = 0;

                                            // DEBUG
                                            Logger.GetInstance().logLine("DEBUG - Improve on swapVehicle");

                                        }
                                        // If new distance is not short
                                        else
                                        {
                                            // Undo swap
                                            solution.customersPaths[vehicle1][cluster1] = cluster1Swp;
                                            solution.customersPaths[vehicle2][cluster2] = cluster2Swp;

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

    }
}
