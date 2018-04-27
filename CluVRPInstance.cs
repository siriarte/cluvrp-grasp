using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace cluvrp_grasp
{
    class NodePoint
    {
        public int x { get; }
        public int y { get; }

        public NodePoint(int x, int y)
        {
            this.x = x;
            this.y = y;
        }
    }
    
    class CluVRPInstance
    {
        public string file_name { get; set; }
        public string name { get; set; }
        public string comment { get; set; }
        public int dimension { get; set; }
        public int vehicles { get; set; }
        public int gvrp_sets { get; set; }
        public int capacity { get; set; }
        public string edge_weight_type { get; set; }
        public NodePoint[] nodes { get; set; }
        public int[][] clusters { get; set; }
        public int[] clusters_demand { get; set; }
        public int depot { get; set; }

        public double[][] clustersDistanceMatrix { set; get; }
        public double[][] customersDistanceMatrix { set; get; }
        public double[] suffleRandomAverageClusterDistance { set; get; }

        public CluVRPInstance(string file_name, string name, string comment, int dimension, int vehicules, int gvrp_sets, 
            int capacity, string edge_weight_type, NodePoint[] nodes, int[][] clusters, int[] clusters_demand, int depot)
        {
            this.file_name = file_name;
            this.name = name;
            this.comment = comment;
            this.dimension = dimension;
            this.vehicles = vehicules;
            this.gvrp_sets = gvrp_sets;
            this.capacity = capacity;
            this.edge_weight_type = edge_weight_type;
            this.nodes = nodes;
            this.clusters = clusters;
            this.clusters_demand = clusters_demand;
            this.depot = depot;

            calculateCustomersDistanceMatrix();
            calculateClustersDistanceMatrix();
            calculateSuffleAverageDistance();
        }

        /*
* Calculate the intercluster distances 
* Use the shortest edge between two clusters as an approximation for the inter-cluster distance
*           
*/
        public void calculateClustersDistanceMatrix()
        {
            // Variables from instances 
            int[][] clusters = this.clusters;
            int numberOfClusters = clusters.Length;

            // Matrix to return
            clustersDistanceMatrix = new double[numberOfClusters][];

            for (int clusterIt1 = 0; clusterIt1 < numberOfClusters; clusterIt1++)
            {
                // Create double[]
                clustersDistanceMatrix[clusterIt1] = new double[numberOfClusters];

                // First cluster to iterate
                int[] cluster1 = clusters[clusterIt1];

                for (int clusterIt2 = 0; clusterIt2 < numberOfClusters; clusterIt2++)
                {
                    // Second cluster to iterate
                    int[] cluster2 = clusters[clusterIt2];

                    // Calculate best distance between 2 cluster
                    double distance = bestDistanceBetween2Clusters(cluster1, cluster2);

                    // Update matrix value
                    clustersDistanceMatrix[clusterIt1][clusterIt2] = distance;
                }
            }
        }

        /*
         * 
         * Calculte min distance between two clusters
         * We use the shortest edge between two clusters as an approximation 
         * for the inter-cluster distance
         *
         */
        private double bestDistanceBetween2Clusters(int[] cluster1, int[] cluster2)
        {

            // Define min distance
            double minDistance = double.MaxValue;

            // For each customer in cluster 1
            for (int i = 0; i < cluster1.Length; i++)
            {
                // Customer 1
                int customer1 = cluster1[i];

                // For each customer in cluster 2
                for (int j = 0; j < cluster2.Length; j++)
                {
                    // Customer 2
                    int customer2 = cluster2[j];

                    // Get distance between customer 1 and customer 2
                    double distance = customersDistanceMatrix[customer1][customer2];

                    // Update min distance
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                    }
                }
            }

            // Return min distance
            return minDistance;
        }

        // Return a customers distance matrix
        private void calculateCustomersDistanceMatrix()
        {
            customersDistanceMatrix = new double[this.dimension + 1][];
            NodePoint[] nodesPosition = this.nodes;

            for (int i = 0; i < this.dimension; i++)
            {
                customersDistanceMatrix[i + 1] = new double[this.dimension + 1];
                for (int j = 0; j < this.dimension; j++)
                {
                    customersDistanceMatrix[i + 1][j + 1] = Functions.distance(nodesPosition[i].x, nodesPosition[i].y, nodesPosition[j].x, nodesPosition[j].y);
                }
            }
        }

        // Calculate random average distance on a cluster
        public void calculateSuffleAverageDistance()
        {
            suffleRandomAverageClusterDistance = new double[clusters.Length];
            for (int clusterIt = 0; clusterIt < clusters.Length; clusterIt++)
            {
                int iterations = clusters[clusterIt].Length * 2;
                List<int> clusterList = clusters[clusterIt].OfType<int>().ToList();
                double totalDistance = 0;
                Random rng = new Random();
                for (int it = 0; it < iterations; it++)
                {
                    Functions.Shuffle(rng, clusterList);
                    totalDistance += interClusterPathDistance(clusterList, customersDistanceMatrix);
                }
                suffleRandomAverageClusterDistance[clusterIt] = totalDistance * 1.0 / iterations;
            }
        }
        
        // Calculate cumplete distance on customerPath
        private static double interClusterPathDistance(List<int> customerList, double[][] costumerDistanceMatrix)
        {
            double distance = 0;
            for (int i = 0; i + 1 < customerList.Count; i++)
            {
                distance += costumerDistanceMatrix[customerList[i]][customerList[i + 1]];
            }
            return distance;
        }
    }

}