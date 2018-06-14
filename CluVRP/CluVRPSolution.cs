﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;

namespace cluvrp_grasp
{
    class CluVRPSolution
    {
        // For cluster problem
        public List<int>[] clusterRouteForVehicule { set; get; }
        public double totalClusterRouteDistance { set; get; }
        public int[] vehicleRemSpace { set; get; }
        public int[] fitAlgorithmCounter { set; get; }
        public FitAlgorithm fitUsed { set; get; }
        public List<LocalSearch> bestClusterLSOrder { set; get; }
        public List<LocalSearch> bestCustomerLSOrder { set; get; }

        // For customer problem
        public List<int>[][] customersPaths { set; get; }
        public double[] _vehiculeRouteDistance;

        // For customer weak problem
        public List<int>[] customersWeakRoute { get; set; }

        // Setter for vehicle to sum the total distance
        public double[] vehiculeRouteDistance
        {
            set
            {
                _vehiculeRouteDistance = value;
                //totalCustomerRouteDistance = _vehiculeRouteDistance.Sum();
             }

            get
            {
                return _vehiculeRouteDistance;
            }
        }

        public double totalCustomerRouteDistance
        {
            get
            {
                if (_vehiculeRouteDistance == null) return double.MaxValue;
                return _vehiculeRouteDistance.Sum();
            }
        }

        // Constructor
        public CluVRPSolution()
        {
            totalClusterRouteDistance = double.MaxValue;
        }

        // Set cluster solution
        public void setClusterSolution(List<int>[] clusterRouteForVehicule, int[] vehicleRemSpace, double totalClusterRouteDistance)
        {
            this.clusterRouteForVehicule = clusterRouteForVehicule;
            this.vehicleRemSpace = vehicleRemSpace;
            this.totalClusterRouteDistance = totalClusterRouteDistance;
        }

        // Set customer solution
        public void setCostumerSolution(List<int>[][] customersPaths, double[] vehiculeRouteDistance)
        {
            this.customersPaths = customersPaths;
            this.vehiculeRouteDistance = vehiculeRouteDistance;
        }

        // Set customer weak solution
        public void setWeakCostumerSolution(List<int>[] customersWeakRoute, double[] vehiculeRouteDistance)
        {
            this.customersWeakRoute = customersWeakRoute;
            this.vehiculeRouteDistance = vehiculeRouteDistance;
        }

        // Cluster solution verification
        public void verifyClusterSolution(CluVRPInstance instance)
        {
            // All cluster was visited
            bool wasVisited = false;
            for (int cluster = 0; cluster < instance.clusters.Length; cluster++)
            {
                for (int vehicle = 0; vehicle < clusterRouteForVehicule.Length; vehicle++)
                {
                    if (clusterRouteForVehicule[vehicle].Contains(cluster))
                    {
                        wasVisited = true;
                        break;
                    }
                }
                Debug.Assert(wasVisited);
                wasVisited = false;
            }

            // Number of clusters visited is correct
            int totalLength = 0;
            for (int vehicle = 0; vehicle < clusterRouteForVehicule.Length; vehicle++)
            {
                totalLength += clusterRouteForVehicule[vehicle].Count;
            }
            Debug.Assert(instance.clusters.Length == totalLength - (2 * clusterRouteForVehicule.Length) + 1);

            // Vehicle remmaining capacity is correct respect to cluster demand
            int[] clusterDemand = instance.clusters_demand;
            int totalDemand = 0;

            // Sum the total demand for vehicle
            for (int vehicle = 0; vehicle < clusterRouteForVehicule.Length; vehicle++)
            {
                int totalDemandOnVehicle = 0;
                for (int clusterIt = 0; clusterIt < clusterRouteForVehicule[vehicle].Count; clusterIt++)
                {
                    int cluster = clusterRouteForVehicule[vehicle][clusterIt];
                    totalDemandOnVehicle += clusterDemand[cluster];
                }

                // Sum the total demand of all vehicles
                totalDemand += totalDemandOnVehicle;
                Debug.Assert(instance.capacity - totalDemandOnVehicle >= 0);
            }
            Debug.Assert(totalDemand == clusterDemand.Sum());
        }

        // Customer solution verification
        public void verifyCustomerSolution(CluVRPInstance instance)
        {
            // Verify number of vehicles
            int numberOfVehicles = instance.vehicles;
            Debug.Assert(customersPaths.Length == numberOfVehicles);
            Debug.Assert(vehiculeRouteDistance.Length == numberOfVehicles);

            // Verify number of clusters
            int numberOfClusters = instance.clusters.Length;
            int clustersCounter = 0;
            for (int i = 0; i < customersPaths.Length; i++)
            {
                clustersCounter += customersPaths[i].Length;
            }
            Debug.Assert(numberOfClusters == clustersCounter - (customersPaths.Length * 2) + 1);

            // Verify number of customers
            int numberOfCustomers = instance.dimension;
            int customersCounter = 0;
            for (int i = 0; i < customersPaths.Length; i++)
            {
                for (int j = 0; j < customersPaths[i].Length; j++)
                {
                    customersCounter += customersPaths[i][j].Count;
                }
            }
            Debug.Assert(numberOfCustomers == customersCounter - (numberOfVehicles * 2) + 1);

            // All clusters are correct
            List<int>[] vehicleRoute = clusterRouteForVehicule;
            for (int vehicle = 0; vehicle < vehicleRoute.Length; vehicle++)
            {
                List<int> clusterList = vehicleRoute[vehicle];
                for (int clusterIt = 0; clusterIt < clusterList.Count; clusterIt++)
                {
                    int clusterNumber = clusterList[clusterIt];
                    List<int> cluster = customersPaths[vehicle][clusterIt].ToList<int>();
                    List<int> clusterInstance = instance.clusters[clusterNumber].ToList<int>();
                    bool containsAll = Functions.ContainsAllItems(cluster, clusterInstance);
                    Debug.Assert(containsAll && cluster.Count == clusterInstance.Count);
                }
            }

            // Total distance is correct
            Debug.Assert(Math.Truncate(totalCustomerRouteDistance) == Math.Truncate(Functions.calculateTotalTravelDistance(customersPaths, instance.customersDistanceMatrix)));

        }

        // Customer solution verification
        public void verifyCustomerWeakSolution(CluVRPInstance instance)
        {
            // Verify number of vehicles
            int numberOfVehicles = instance.vehicles;
            Debug.Assert(customersWeakRoute.Length == numberOfVehicles);
            Debug.Assert(vehiculeRouteDistance.Length == numberOfVehicles);

            // Verify number of customers and all paths start and end on depot
            int numberOfCustomers = instance.dimension;
            int customersCounter = 0;
            for (int i = 0; i < customersWeakRoute.Length; i++)
            {
                customersCounter += customersWeakRoute[i].Count;
                Debug.Assert(customersWeakRoute[i][0] == 1 && customersWeakRoute[i][customersWeakRoute[i].Count - 1] == 1);
            }
            Debug.Assert(numberOfCustomers == customersCounter - (numberOfVehicles * 2) + 1);

            // All clusters are correct
            List<int>[] vehicleRoute = clusterRouteForVehicule;
            for (int vehicle = 0; vehicle < vehicleRoute.Length; vehicle++)
            {
                List<int> clusterList = vehicleRoute[vehicle];
                int allClustersSizeSum = 0;
                for (int clusterIt = 0; clusterIt < clusterList.Count; clusterIt++)
                {
                    int clusterNumber = clusterList[clusterIt];
                    List<int> cluster = customersWeakRoute[vehicle].ToList<int>();
                    List<int> clusterInstance = instance.clusters[clusterNumber].ToList<int>();
                    allClustersSizeSum += clusterInstance.Count;
                    bool containsAll = Functions.ContainsAllItems(cluster, clusterInstance);
                    Debug.Assert(containsAll);
                }
                Debug.Assert(customersWeakRoute[vehicle].Count == allClustersSizeSum);
            }

            // Total distance is correct
            Debug.Assert(Math.Truncate(totalCustomerRouteDistance) == Math.Truncate(Functions.calculateCustomerTotalTravelDistanceForVehicle(customersWeakRoute, instance.customersDistanceMatrix)));

        }

        // Print solution information
        public void printSolution()
        {
            string tittle = "RESULT:";
            string clusterDistance = totalClusterRouteDistance.ToString();
            string totalDistance = totalCustomerRouteDistance.ToString();
            string vehiculeDistance = string.Join(" -- ", vehiculeRouteDistance);

            Logger.GetInstance().logLine(tittle);
            Logger.GetInstance().logLine(clusterDistance);
            Logger.GetInstance().logLine(totalDistance);
            Logger.GetInstance().logLine(vehiculeDistance);

            for (int i = 0; i < customersPaths.Length; i++)
            {
                string vehiculeTittle = "Route for vehicle: " + i;
                Logger.GetInstance().logLine(vehiculeTittle);

                for (int clusterIt = 0; clusterIt < customersPaths[i].Length; clusterIt++)
                {
                    string circuit = string.Join(", ", customersPaths[i][clusterIt]);
                    Logger.GetInstance().logLine(circuit);
                }
            }

        }

        // Check demand attended by vehicles is correct respect the capacity - FOR DEBUG
        public static void checkDemand(CluVRPInstance instance, List<int>[] clusterRouteForVehicule, int[] vehicleRemSpace)
        {    
            // Vehicle remmaining capacity is correct respect to cluster demand
            int[] clusterDemand = instance.clusters_demand;
            int totalDemand = 0;

            // Sum the total demand for vehicle
            for (int vehicle = 0; vehicle < clusterRouteForVehicule.Length; vehicle++)
            {
                int totalDemandOnVehicle = 0;
                for (int clusterIt = 0; clusterIt < clusterRouteForVehicule[vehicle].Count; clusterIt++)
                {
                    int cluster = clusterRouteForVehicule[vehicle][clusterIt];
                    totalDemandOnVehicle += clusterDemand[cluster];
                }

                // Sum the total demand of all vehicles
                totalDemand += totalDemandOnVehicle;
                
                // Asserts
                Debug.Assert(instance.capacity - totalDemandOnVehicle < 0);
                Debug.Assert(instance.capacity - totalDemandOnVehicle != vehicleRemSpace[vehicle] || vehicleRemSpace[vehicle] < 0);
            }

            // Assert
            Debug.Assert(totalDemand == clusterDemand.Sum());
        }
         
    }
}