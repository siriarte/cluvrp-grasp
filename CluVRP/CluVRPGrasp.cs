using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace cluvrp_grasp
{
    enum CluVRPVersion { TwoPhase, Complete }
    enum CluVRPType { Normal, Weak }

    class CluVRPGrasp
    {       

        // Main GRASP handle main iteration depending if use Complete version or TwoPhase
        public static CluVRPSolution Grasp(CluVRPInstance instance, Parameters parameters)
        {
            // For best solution
            CluVRPSolution bestSolution = new CluVRPSolution();
          
            // Main cycle
            int iterator = 0;
            while(iterator < parameters.CluVRP_GRASPIterations)
            {
                // New Grasp for Cluster level instance
                ClusterGRASP clusterGrasp = new ClusterGRASP(instance, parameters);

                // Execute Grasp procedure
                CluVRPSolution cluVRPSolution = clusterGrasp.Grasp();

                // If solutions is not available continue with next iteration
                if (cluVRPSolution.clusterRouteForVehicule == null)
                {
                    iterator++;
                    continue;
                }
          
                // Verify if cluster solution is correct
                cluVRPSolution.verifyClusterSolution(instance);

                // New Grasp for Cluster level instance
                CustomerGRASP customerGrasp = new CustomerGRASP(instance, cluVRPSolution, parameters);

                // Execute Grasp procedure
                customerGrasp.Grasp();

                SwapClusters(cluVRPSolution, instance);

                swapVehicle(cluVRPSolution, instance);

                // Verify if customer solution is correct
                cluVRPSolution.verifyCustomerSolution(instance);
                

                // Update best solution
                if (cluVRPSolution.totalCustomerRouteDistance < bestSolution.totalCustomerRouteDistance)
                {
                    bestSolution = cluVRPSolution;
                }
                
                // Increase iterator
                iterator++;
            }
                        
           // Return best solution
           return bestSolution;
        }

        // Main GRASP (for Weak cluster constrains) handle main iteration depending if use Complete version or TwoPhase
        public static CluVRPSolution GraspForWeakCustomer(CluVRPInstance instance, Parameters parameters)
        {
            // For best solution
            CluVRPSolution bestSolution = new CluVRPSolution();

            // Main cycle
            int iterator = 0;
            while (iterator < parameters.CluVRP_GRASPIterations)
            {
                // New Grasp for Cluster level instance
                ClusterGRASP clusterGrasp = new ClusterGRASP(instance, parameters);

                // Execute Grasp procedure
                CluVRPSolution cluVRPSolution = clusterGrasp.Grasp();

                // If solutions is not avalaible continue next iteration
                if (cluVRPSolution.clusterRouteForVehicule == null)
                {
                    iterator++;
                    continue;
                }
                
                // Verify if cluster solution is correct
                cluVRPSolution.verifyClusterSolution(instance);

                // New Grasp for Cluster level instance
                CustomerWeakGRASP customerGrasp = new CustomerWeakGRASP(instance, cluVRPSolution, parameters);

                // Execute Grasp procedure
                customerGrasp.Grasp();

                // Verify if customer solution is correct
                cluVRPSolution.verifyCustomerWeakSolution(instance);
                
                // Update best solution
                if (cluVRPSolution.totalCustomerRouteDistance < bestSolution.totalCustomerRouteDistance)
                {
                    bestSolution = cluVRPSolution;
                }

                // Increase iterator
                iterator++;
            }

            // Return best solution
            return bestSolution;
        }

        public static void SwapClusters(CluVRPSolution solution, CluVRPInstance instance)
        {
            for (int vehicle = 0; vehicle < solution.customersPaths.Length; vehicle++)
            {
                for (int clusterIt1 = 1; clusterIt1 + 1 < solution.customersPaths[vehicle].Length; clusterIt1++)
                {
                    for (int clusterIt2 = 1; clusterIt2 + 1 < solution.customersPaths[vehicle].Length; clusterIt2++)
                    {
                        if (clusterIt1 != clusterIt2)
                        {
                            List<int> cluster1Last = solution.customersPaths[vehicle][clusterIt1-1];
                            List<int> cluster1 = solution.customersPaths[vehicle][clusterIt1];
                            List<int> cluster1Next = solution.customersPaths[vehicle][clusterIt1 +1];

                            List<int> cluster2Last = solution.customersPaths[vehicle][clusterIt2 - 1];
                            List<int> cluster2 = solution.customersPaths[vehicle][clusterIt2];
                            List<int> cluster2Next = solution.customersPaths[vehicle][clusterIt2 + 1];

                            double oldDistance = Functions.calculateTotalTravelDistance(solution.customersPaths, instance.customersDistanceMatrix, vehicle);
                            solution.customersPaths[vehicle][clusterIt1] = cluster2;
                            solution.customersPaths[vehicle][clusterIt2] = cluster1;
                            Functions.Swap(solution.clusterRouteForVehicule[vehicle], clusterIt1, clusterIt2);
                            
                            double newDistance = Functions.calculateTotalTravelDistance(solution.customersPaths, instance.customersDistanceMatrix, vehicle);

                            if (oldDistance <= newDistance)
                            {
                                solution.customersPaths[vehicle][clusterIt1] = cluster1;
                                solution.customersPaths[vehicle][clusterIt2] = cluster2;
                                Functions.Swap(solution.clusterRouteForVehicule[vehicle], clusterIt1, clusterIt2);
                            }
                            else
                            {
                                solution.vehiculeRouteDistance[vehicle] = newDistance;
                            }

                        }
                    }
                }
            }
        }

        // Try to swap all the clusters (one bye one) for all vehicles (one by one)
        public static void swapVehicle(CluVRPSolution solution, CluVRPInstance instance)
        {
            int[] clusterDemand = instance.clusters_demand;

            for (int iterations = 0; iterations < 1; iterations++)
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
                                        double oldDistanceVehicle1 = Functions.calculateCustomerTotalTravelDistanceForVehicle(solution.customersPaths[vehicle1], instance.clustersDistanceMatrix);
                                        double oldDistanceVehicle2 = Functions.calculateCustomerTotalTravelDistanceForVehicle(solution.customersPaths[vehicle2], instance.clustersDistanceMatrix);

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
            }

            //End
            return;
        }

    }
}
