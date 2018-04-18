using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace cluvrp_grasp
{
    static class Functions
    {
        // Return a customers distance matrix
        static public double[][] customersDistanceMatrix(CluVRPInstance instance)
        {
            double[][] nodesDistanceMatrix = new double[instance.dimension()][];
            NodePoint[] nodesPosition = instance.nodes();

            for (int i = 0; i < instance.dimension(); i++)
            {
                nodesDistanceMatrix[i] = new double[instance.dimension()];
                for (int j = 0; j < instance.dimension(); j++)
                {
                    nodesDistanceMatrix[i][j] = distance(nodesPosition[i].getX(), nodesPosition[i].getY(), nodesPosition[j].getX(), nodesPosition[j].getY());
                }
            }
            return nodesDistanceMatrix;
        }
        
        // Distance function
        static public double distance(double x1, double y1, double x2, double y2)
        {
            //return 1;
            return Math.Sqrt(Math.Pow(x1 - x2, 2) + Math.Pow(y1 - y2, 2));
        }

        // Return an array with the index of the input array sorted
        static private int[] arraySortedByIndex(int[] arr)
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
                    if (arr[ret[j]] >= arr[ret[i]])
                    {
                        int temp = ret[i];
                        ret[i] = ret[j];
                        ret[j] = temp;
                    }
                }
            }
            return ret;
        }

        // Swap
        public static void Swap<T>(IList<T> list, int indexA, int indexB)
        {
            T tmp = list[indexA];
            list[indexA] = list[indexB];
            list[indexB] = tmp;
        }
         
        // Select a element from a RCL list with random criteria
        public static int selectRandomElement(List<int> list)
        {
            Random rnd = new Random();
            int rndIndex = rnd.Next(0, list.Count);
            return list[rndIndex];
        }

        // Calculate the total travel (visiting all the custer by each vehicle)
        static public double calculateTotalTravelDistance(List<int>[][] customersCircuit, double[][] customersDistanceMatrix)
        {
            double distance = 0;
            int customer1 = 0;
            int customer2 = 0;

            for (int vehicle = 0; vehicle < customersCircuit.Length; vehicle++)
            {
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
                        int customerNextCustomer = customersCircuit[vehicle][clusterIt + 1][0];
                        distance += customersDistanceMatrix[customer2][customerNextCustomer];
                    }

                }
            }
            return distance;
        }

        // Calculate the total travel visiting all the custer by ONE vehicle)
        static public double calculateTotalTravelDistance(List<int>[][] customersCircuit, double[][] customersDistanceMatrix, int vehicle)
        {
            double distance = 0;
            int customer1 = 0;
            int customer2 = 0;

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
                    int customerNextCustomer = customersCircuit[vehicle][clusterIt + 1][0];
                    distance += customersDistanceMatrix[customer2][customerNextCustomer];
                }

            }
            
            return distance;
        }
                
        // Populate array
        public static void Populate<T>(this T[] arr, T value)
        {
            for (int i = 0; i < arr.Length; i++)
            {
                arr[i] = value;
            }
        }

    }
}
