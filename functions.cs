using System;
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
