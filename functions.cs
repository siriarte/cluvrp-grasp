﻿using System;
using System.Collections.Generic;
using System.Linq;

namespace cluvrp_grasp
{
    static class Functions
    {
        // Return an array with the index of the input array sorted
        static public int[] arraySortedByIndex(int[] arr)
        {
            int[] ret = new int[arr.Length];
            for (int i = 0; i < arr.Length; i++)
            {
                ret[i] = i;
            }

            for (int i = 0; i < arr.Length; i++)
            {
                for (int j = i + 1; j < arr.Length; j++)
                {
                    if (arr[ret[j]] <= arr[ret[i]])
                    {
                        int temp = ret[i];
                        ret[i] = ret[j];
                        ret[j] = temp;
                    }
                }
            }
            return ret;
        }

        // Return an array with the soreted index of the input array
        static public void sortClusterByDemand(List<int> clusters, int[] demand)
        {
            for (int i = 0; i < clusters.Count; i++)
            {
                for (int j = i + 1; j < clusters.Count; j++)
                {
                    if (demand[clusters[i]] >= demand[clusters[j]])
                    {
                        Functions.Swap(clusters, i, j);
                    }
                }
            }
        }

        // Swap
        public static void Swap<T>(IList<T> list, int indexA, int indexB)
        {
            T tmp = list[indexA];
            list[indexA] = list[indexB];
            list[indexB] = tmp;
        }
         
        // Select a element from a list with random criteria
        public static int selectRandomElement(List<int> list)
        {
            Random rnd = new Random();
            int rndIndex = rnd.Next(0, list.Count);
            return list[rndIndex];
        }

        // Calculate the total travel (visiting all the custer and customers by each vehicle)
        static public double calculateTotalTravelDistance(List<int>[][] customersCircuit, double[][] customersDistanceMatrix)
        {
            double distance = 0;

            for (int vehicle = 0; vehicle < customersCircuit.Length; vehicle++)
            {
                int customer1 = 1;
                int customer2 = 1;

                for (int clusterIt = 0; clusterIt < customersCircuit[vehicle].Length; clusterIt++)
                {
                    for (int customerIt = 0; customerIt + 1 < customersCircuit[vehicle][clusterIt].Count; customerIt++)
                    {
                        customer1 = customersCircuit[vehicle][clusterIt][customerIt];
                        customer2 = customersCircuit[vehicle][clusterIt][customerIt + 1];
                        distance += customersDistanceMatrix[customer1][customer2];
                    }

                    if (clusterIt + 1 < customersCircuit[vehicle].Length)
                    {
                        int finalCustomerIdx = customersCircuit[vehicle][clusterIt].Count - 1;
                        int finalCustomer = customersCircuit[vehicle][clusterIt][finalCustomerIdx];
                        int initialCustomer = customersCircuit[vehicle][clusterIt + 1][0];
                        distance += customersDistanceMatrix[finalCustomer][initialCustomer];
                    }
                }
            }

            return distance;
        }

        // Calculate the total travel visiting all the custer by ONE vehicle
        static public double calculateTotalTravelDistance(List<int>[][] customersCircuit, double[][] customersDistanceMatrix, int vehicle)
        {
            double distance = 0;
            int customer1 = 1;
            int customer2 = 1;

            for (int clusterIt = 0; clusterIt < customersCircuit[vehicle].Length; clusterIt++)
            {
                for (int customerIt = 0; customerIt + 1 < customersCircuit[vehicle][clusterIt].Count; customerIt++)
                {
                    customer1 = customersCircuit[vehicle][clusterIt][customerIt];
                    customer2 = customersCircuit[vehicle][clusterIt][customerIt + 1];
                    distance += customersDistanceMatrix[customer1][customer2];
                }

                if (clusterIt + 1 < customersCircuit[vehicle].Length)
                {
                    int finalCustomerIdx = customersCircuit[vehicle][clusterIt].Count - 1;
                    int finalCustomer = customersCircuit[vehicle][clusterIt][finalCustomerIdx];
                    int initialCustomer = customersCircuit[vehicle][clusterIt + 1][0];
                    distance += customersDistanceMatrix[finalCustomer][initialCustomer];
                }
            }
            
            return distance;
        }
  
        // Calculate the total cluster distance of the cluster travel for all vehicles
        public static double calculateTotalClusterTravelDistance(List<int>[] travel, double[][] clustersDistanceMatrix)
        {
            // Set variables
            double totalDistance = 0;

            // Iterate each vehicle
            for (int vehicleNumber = 0; vehicleNumber < travel.Length; vehicleNumber++)
            {
                totalDistance += calculateClusterTravelDistanceForVehicle(travel[vehicleNumber], clustersDistanceMatrix);
            }

            // Return total distance
            return totalDistance;
        }
  
        // Calculate the total cluster distance of a vehicle travel
        public static double calculateClusterTravelDistanceForVehicle(List<int> travel, double[][] clustersDistanceMatrix)
        {
            // Set variables
            double totalDistance = 0;

            // Iterate each cluster on vehicle route
            for (int clusterIt = 0; clusterIt + 1 < travel.Count; clusterIt++)
            {
                int fromCluster = travel[clusterIt];
                int ToCluster = travel[clusterIt + 1];
                totalDistance += clustersDistanceMatrix[fromCluster][ToCluster];
            }

            // Return total distance
            return totalDistance;
        }

        // Calculate the RANDOM-SHUFFLE-ESTIMATED cluster distance of the cluster travel for all vehicles
        public static double estimateTotalClusterTravelDistance(List<int>[] travel, double[][] clustersDistanceMatrix, double[] suffleAverageClusterDistance)
        {
            // Init distance
            double totalDistance = 0;

            // Iterate each cluster on vehicle route
            for (int vehicle = 0; vehicle < travel.Length; vehicle++)
            {
                totalDistance += estimateClusterTravelDistanceForVehicle(travel[vehicle], clustersDistanceMatrix, suffleAverageClusterDistance);
            }

            // Return total distance
            return totalDistance;
        }

        // Calculate the RANDOM-SHUFFLE-ESTIMATED cluster distance of a vehicle travel
        public static double estimateClusterTravelDistanceForVehicle(List<int> travel, double[][] clustersDistanceMatrix, double[] suffleAverageClusterDistance)
        {
            // Set variables
            double totalDistance = 0;

            // Iterate each cluster on vehicle route
            for (int clusterIt = 0; clusterIt + 1 < travel.Count; clusterIt++)
            {
                int fromCluster = travel[clusterIt];
                int ToCluster = travel[clusterIt + 1];
                totalDistance += clustersDistanceMatrix[fromCluster][ToCluster];
                totalDistance += suffleAverageClusterDistance[fromCluster];
            }

            // Return total distance
            return totalDistance;
        }
        
        // Calculte the max distance between the last cluster visited (of all vehicles) and toCluster      
        public static double maxClusterDistance(List<int>[] clusterRouteForVehicle, int toCluster, double[][] clustersDistanceMatrix)
        {
            double ret = double.MinValue;
            for (int i = 0; i < clusterRouteForVehicle.Length; i++)
            {
                int lastClusterIndex = clusterRouteForVehicle[i].Count - 1;
                int lastCluster = (int)clusterRouteForVehicle[i][lastClusterIndex];
                ret = Math.Max(ret, clustersDistanceMatrix[lastCluster][toCluster]);
            }
            return ret;
        }

        // Calculte the min distance between the last cluster visited (of all vehicles) and toCluster    
        public static double minClusterDistance(List<int>[] clusterRouteForVehicle, int toCluster, double[][] clustersDistanceMatrix)
        {
            double ret = double.MaxValue;
            for (int i = 0; i < clusterRouteForVehicle.Length; i++)
            {
                int lastClusterIndex = clusterRouteForVehicle[i].Count - 1;
                int lastCluster = (int)clusterRouteForVehicle[i][lastClusterIndex];
                ret = Math.Min(ret, clustersDistanceMatrix[lastCluster][toCluster]);
            }
            return ret;
        }

        // Calculte the max distance between the last cluster visited (of all vehicles) and toCluster
        public static double maxCustomerDistance(List<int> customers, int toCustomer, double[][] customersDistanceMatrix)
        {
            double ret = double.MinValue;
            for (int i = 0; i < customers.Count; i++)
            {
                ret = Math.Max(ret, customersDistanceMatrix[customers[i]][toCustomer]);

            }
            return ret;
        }

        // Calculte the min distance between the last cluster visited (of all vehicles) and toCluster
        public static double minCustomerDistance(List<int> customers, int toCustomer, double[][] customersDistanceMatrix)
        {
            double ret = double.MaxValue;
            for (int i = 0; i < customers.Count; i++)
            {
                ret = Math.Min(ret, customersDistanceMatrix[customers[i]][toCustomer]);
            }
            return ret;
        }

        // Verify is cluster route is valid (start and end in depot)
        public static bool isValidClusterRoute(List<int> route)
        {
            if (route.Count > 0)
            {
                return (route[0] == 0 && route[route.Count - 1] == 0);
            }
            return false;
        }

        // Populate array with 'value'
        public static void Populate<T>(this T[] arr, T value)
        {
            for (int i = 0; i < arr.Length; i++)
            {
                arr[i] = value;
            }
        }

        // Verify that list a contains all item of list b   
        public static bool ContainsAllItems(List<int> a, List<int> b)
        {
            return !b.Except(a).Any();
        }

        // Suffle array
        public static void Shuffle<T>(Random rng, List<T> array)
        {
            int n = array.Count;
            while (n > 1)
            {
                int k = rng.Next(n--);
                T temp = array[n];
                array[n] = array[k];
                array[k] = temp;
            }
        }

        // Distance function
        public static double distance(double x1, double y1, double x2, double y2)
        {
            return (Math.Sqrt(Math.Pow(x1 - x2, 2) + Math.Pow(y1 - y2, 2)));
        }

    }
}
