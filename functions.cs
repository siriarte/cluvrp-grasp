﻿using System;
using System.Collections.Generic;
using System.Linq;
using Newtonsoft.Json;
using System.IO;

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

        // Return a string with all parameters in parameters
        public static string parametersToString(Parameters parameters)
        {
            // Init variables
            string separator = "\n";
            string ret = "";

            // For GRASP
            ret += "CLUVRP GRASP VERSION = " + parameters.CluVRP_Version + separator;
            ret += "CLUVRP GRASP ITERATIONS = " + parameters.CluVRP_GRASPIterations + separator;
            ret += "CLUVRP CLUSTER GRASP ITERATIONS = " + parameters.Cluster_GRASPIterations + separator;
            ret += "CLUVRP CUSTOMER GRASP ITERATIONS = " + parameters.Customer_GRASPIterations + separator;
            ret += "CLUVRP MAIN LS ITERATIONS = " + parameters.CluVRP_LS_Main_Iterations + separator;
            ret += "CLUVRP FINAL SWAP CLUSTER = " + parameters.CluVRP_LS_SwapClusters + separator;
            ret += "CLUVRP FINAL SWAP VEHICLE = " + parameters.CluVRP_LS_SwapVehicle + separator;
            ret += separator;

            // For cluster
            ret += "CLUSTER FIT ALGORITHM = " + parameters.Cluster_FitAlgoritm + separator;
            ret += "CLUSTER ALPHA DISTANCE = " + parameters.Cluster_AlphaDistance.ToString("0.0") + separator;
            ret += "CLUSTER ALPHA CAPACITY/BACKTODEPOT = " + parameters.Cluster_AlphaCapacity.ToString("0.0") + separator;
            ret += "CLUSTER LS TWO-OPT = " + parameters.Cluster_LS_TwoOpt_Iterations + separator;
            ret += "CLUSTER LS RELOCATE = " + parameters.Cluster_LS_Relocate_Iterations + separator;
            ret += "CLUSTER LS EXCHANGE = " + parameters.Cluster_LS_Exchange_Iterations + separator;
            ret += "CLUSTER LS INSERT VEHICLE = " + parameters.Cluster_LS_InsertVehicle.ToString() + separator;
            ret += "CLUSTER LS SWAP VEHICLE = " + parameters.Cluster_LS_SwapVehicle.ToString() + separator;
            ret += "CLUSTER LS SWAP CLUSTERS = " + parameters.Cluster_LS_SwapClusters.ToString() + separator;
            ret += "CLUSTER LS ORDER = " + '[' + string.Join(",", parameters.Cluster_LS_Order) + ']' + separator;
            ret += separator;

            // For customer
            ret += "CUSTOMER ALPHA = " + parameters.Customer_Alpha.ToString("0.0") + separator;
            ret += "CUSTOMER LS TWO-OPT = " + parameters.Customer_LS_TwoOpt_Iterations + separator;
            ret += "CUSTOMER LS RELOCATE = " + parameters.Customer_LS_Relocate_Iterations + separator;
            ret += "CUSTOMER LS EXCHANGE = " + parameters.Customer_LS_Exchange_Iterations + separator;
            ret += "CUSTOMER LS SWAP CUSTOMERS = " + parameters.Customer_LS_SwapCustomers + separator;
            ret += "CUSTOMER LS ORDER = " + '[' + string.Join(",", parameters.Customer_LS_Order) + ']' + separator;

            // Return string
            return ret;
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
        static public double calculateTotalTravelDistance(List<int>[][] customersCircuit, double[][] customersDistanceMatrix, CluVRPInstance instance)
        {
            double distance = 0;

            for (int vehicle = 0; vehicle < customersCircuit.Length; vehicle++)
            {
                if(instance.instance_type == Instance.GoldenBattarra || instance.instance_type == Instance.GVRP)
                    distance += (int)calculateTotalTravelDistance(customersCircuit, customersDistanceMatrix, vehicle);
                if (instance.instance_type == Instance.GoldenIzquierdo)
                    distance += calculateTotalTravelDistance(customersCircuit, customersDistanceMatrix, vehicle);
            }

            return distance;
        }

        // Calculate the total travel visiting all the custer for ONE vehicle
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

        // Calculate the total customer distance of a vehicle travel
        public static double calculateCustomerTravelDistance(List<int> travel, double[][] customersDistanceMatrix)
        {
            // Set variables
            double totalDistance = 0;

            // Iterate each cluster on vehicle route
            for (int clusterIt = 0; clusterIt + 1 < travel.Count; clusterIt++)
            {
                int fromCluster = travel[clusterIt];
                int ToCluster = travel[clusterIt + 1];
                totalDistance += customersDistanceMatrix[fromCluster][ToCluster];
            }

            // Return total distance
            return totalDistance;
        }

        // Calculate the total customer distance of a all vehicles
        public static double calculateCustomerTotalTravelDistanceForVehicle(List<int>[] travel, double[][] customersDistanceMatrix, CluVRPInstance instance)
        {
            // Set variables
            double totalDistance = 0;

            for (int vehicle = 0; vehicle < travel.Length; vehicle++)
            {
                if (instance.instance_type == Instance.GoldenBattarra || instance.instance_type == Instance.GVRP)
                    totalDistance += (int) calculateCustomerTravelDistance(travel[vehicle], customersDistanceMatrix);
                if (instance.instance_type == Instance.GoldenIzquierdo)
                    totalDistance += calculateCustomerTravelDistance(travel[vehicle], customersDistanceMatrix);

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

        // Calculate cluster path distance with in and out of its
        public static double calculateInOutAndPathDistance(List<int>[] clusterPath, int clusterIdx, double[][] customersDistanceMatrix)
        {
            int lastCluster = clusterIdx - 1;
            int nextCluster = clusterIdx + 1;
            int lastClusterLastCustomer = clusterPath[lastCluster][clusterPath[lastCluster].Count - 1];
            int nextClusterNextCustomer = clusterPath[nextCluster][0];

            int actualClusterFirstCustomer = clusterPath[clusterIdx][0];
            int actualClusterLastCustomer = clusterPath[clusterIdx][clusterPath[clusterIdx].Count - 1];

            double distance = customersDistanceMatrix[lastClusterLastCustomer][actualClusterFirstCustomer];
            distance += customersDistanceMatrix[actualClusterLastCustomer][nextClusterNextCustomer];

            for (int cluster = 0; cluster + 1 < clusterPath[clusterIdx].Count; cluster++)
            {
                int from = clusterPath[clusterIdx][cluster];
                int to = clusterPath[clusterIdx][cluster + 1];
                distance += customersDistanceMatrix[from][to];
            }

            return distance;
        }

        internal static double maxClusterDistanceFromVehicle(List<int> clustersToVisit, int lastClusterOnVehicle, double[][] clustersDistanceMatrix)
        {
            double max = double.MinValue;
            for (int i = 0; i < clustersToVisit.Count; i++)
            {
                max = Math.Max(clustersDistanceMatrix[clustersToVisit[i]][lastClusterOnVehicle], max);
            }
            return max;
        }

        internal static double minClusterDistanceFromVehicle(List<int> clustersToVisit, int lastClusterOnVehicle, double[][] clustersDistanceMatrix)
        {
            double min = double.MaxValue;
            for (int i = 0; i < clustersToVisit.Count; i++)
            {
                min = Math.Min(clustersDistanceMatrix[clustersToVisit[i]][lastClusterOnVehicle], min);
            }
            return min;
        }

        internal static double maxClusterDistanceFromVehicleAndToDepot(List<int> clustersToVisit, int lastClusterOnVehicle, double[][] clustersDistanceMatrix, double alphaBackToDepot)
        {
            double max = double.MinValue;
            for (int i = 0; i < clustersToVisit.Count; i++)
            {
                double dist = clustersDistanceMatrix[clustersToVisit[i]][lastClusterOnVehicle] +
                    alphaBackToDepot * clustersDistanceMatrix[clustersToVisit[i]][0];
                max = Math.Max(dist, max);
            }
            return max;
        }

        internal static double minClusterDistanceFromVehicleAndToDepot(List<int> clustersToVisit, int lastClusterOnVehicle, double[][] clustersDistanceMatrix, double alphaBackToDepot)
        {
            double min = double.MaxValue;
            for (int i = 0; i < clustersToVisit.Count; i++)
            {
                double dist = clustersDistanceMatrix[clustersToVisit[i]][lastClusterOnVehicle] +
                    alphaBackToDepot * clustersDistanceMatrix[clustersToVisit[i]][0];
                min = Math.Min(dist, min);
            }
            return min;
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

        // Array to string
        public static string arrayToString<T>(ICollection<T> collection)
        {
            return '[' + string.Join(" | ", collection) + ']';
        }

        // Load JSON instances solution to a Dicctionary
        public static Dictionary<String, Double> loadInstancesSolution(string jsonFilePath)
        {
            using (StreamReader r = new StreamReader(jsonFilePath))
            {
                string json = r.ReadToEnd();
                Dictionary<String, Double> items = JsonConvert.DeserializeObject<Dictionary<string, double>>(json);
                return items;
            }
        }

        // Create an array of solution for an instances file
        public static double[] createSolutionArrayForInstances(string instanceSetFilePath, string solutionsFilePath)
        {
            // Return value
            double[] arrayOfSolutions = null;

            // Create proccess
            try {
                Dictionary<string, double> instanceAndSolutions = loadInstancesSolution(solutionsFilePath);
                string[] lines = System.IO.File.ReadAllLines(instanceSetFilePath);
                arrayOfSolutions = new double[lines.Length];

                // Get all the instances setted on the file
                int idx = 0;
                foreach (string instanceFilePath in lines)
                {
                    string instanceFileName = Path.GetFileName(instanceFilePath);
                    arrayOfSolutions[idx] = instanceAndSolutions[instanceFileName];
                    idx++;                   
                }

            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }

            // Return array
            return arrayOfSolutions;
        }
    }
}
