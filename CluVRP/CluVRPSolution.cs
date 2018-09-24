using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using System.Text;

namespace cluvrp_grasp
{
    class CluVRPSolution
    {
        // Configuration
        const string PYTHON_PATH = "c:\\util\\Python\\Python36-32\\python.exe";

        // For cluster problem
        public List<int>[] clusterRouteForVehicule { set; get; }
        public double totalClusterRouteDistance { set; get; }
        public int[] vehicleRemSpace { set; get; }
        public int[] fitAlgorithmCounter { set; get; }
        public FitAlgorithm fitUsed { set; get; }
        public List<LocalSearch> bestClusterLSOrder { set; get; }
        public List<LocalSearch> bestCustomerLSOrder { set; get; }
        public Instance instaceType { set; get; }

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
                // If not solution available return max value
                if (_vehiculeRouteDistance == null) return double.MaxValue;

                // If instance is battarra type the solution is an integer
                if(instaceType == Instance.GoldenBattarra || instaceType == Instance.GVRP) { 
                    int sum = 0;
                    for(int vehicle = 0; vehicle < _vehiculeRouteDistance.Length; vehicle++)
                    {
                        sum += (int)_vehiculeRouteDistance[vehicle];
                    }
                    return sum;
                }
               
                // For Izquierdo the solution is double
                return _vehiculeRouteDistance.Sum();
            }
        }

        // Constructor
        public CluVRPSolution(CluVRPInstance instance)
        {
            totalClusterRouteDistance = double.MaxValue;
            instaceType = instance.instance_type;
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
                Debug.Assert(wasVisited, "All clusters are not visited");
                wasVisited = false;
            }

            // Number of clusters visited is correct
            int totalLength = 0;
            for (int vehicle = 0; vehicle < clusterRouteForVehicule.Length; vehicle++)
            {
                totalLength += clusterRouteForVehicule[vehicle].Count;
            }
            Debug.Assert(instance.clusters.Length == totalLength - (2 * clusterRouteForVehicule.Length) + 1, "Number of cluster visited is incorrect");

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
                Debug.Assert(instance.capacity - totalDemandOnVehicle >= 0, "The total demand is more big than the capacity of a vehicle");
            }
            Debug.Assert(totalDemand == clusterDemand.Sum(), "The total demand is more big than the total capacity");

            // Verify if all vehicules visit at least 1 cluster
            // This is necessary for GVRP and GoldelBattarra instances
            if (instance.instance_type == Instance.GVRP || instance.instance_type == Instance.GoldenBattarra)
            {
                for (int vehicle = 0; vehicle < clusterRouteForVehicule.Length; vehicle++)
                {
                    Debug.Assert(clusterRouteForVehicule[vehicle].Count > 2, "There are vehicles that no visit clusters");
                }
            }

        }

        // Customer solution verification
        public void verifyCustomerSolution(CluVRPInstance instance)
        {
            // Verify number of vehicles
            int numberOfVehicles = instance.vehicles;
            Debug.Assert(customersPaths.Length == numberOfVehicles, "The number of vehicles on the  customer path is incorrect");
            Debug.Assert(vehiculeRouteDistance.Length == numberOfVehicles, "The number of vehicles on the route distance is incorrect");

            // Verify number of clusters
            int numberOfClusters = instance.clusters.Length;
            int clustersCounter = 0;
            for (int i = 0; i < customersPaths.Length; i++)
            {
                clustersCounter += customersPaths[i].Length;
            }
            Debug.Assert(numberOfClusters == clustersCounter - (customersPaths.Length * 2) + 1, "The number of cluster in the complete travel is incorrect");

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
                    if (vehicleRoute[vehicle].Count != 2)
                    {
                        Debug.Assert(containsAll && cluster.Count == clusterInstance.Count, "All the clusters on the travel are not correct respect to their clients");
                    }
                }
            }

            // Total distance is correct
            Debug.Assert(Math.Truncate(totalCustomerRouteDistance) == Math.Truncate(Functions.calculateTotalTravelDistance(customersPaths, instance.customersDistanceMatrix, instance)), "The final distance is not correct");

        }

        // Customer solution verification
        public void verifyCustomerWeakSolution(CluVRPInstance instance)
        {
            // Verify number of vehicles
            int numberOfVehicles = instance.vehicles;
            Debug.Assert(customersWeakRoute.Length == numberOfVehicles, "CustomerWeak - The number of vehicles is incorrect respect to customer path");
            Debug.Assert(vehiculeRouteDistance.Length == numberOfVehicles, "CustomerWeak - The number of vehicles is incorrect respect to vehicles distance vector");

            // Verify number of customers and all paths start and end on depot
            int numberOfCustomers = instance.dimension;
            int customersCounter = 0;
            for (int i = 0; i < customersWeakRoute.Length; i++)
            {
                customersCounter += customersWeakRoute[i].Count;
                Debug.Assert(customersWeakRoute[i][0] == 1 && customersWeakRoute[i][customersWeakRoute[i].Count - 1] == 1, "CustomerWeak - There is a customer path that not start or end on the depot");
            }
            Debug.Assert(numberOfCustomers == customersCounter - (numberOfVehicles * 2) + 1, "CustomerWeak - The number of total customers is incorrect on the travel");

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
                    Debug.Assert(containsAll, "CustomerWeak - All the clusters not contains the corrects customers");
                }
                Debug.Assert(customersWeakRoute[vehicle].Count == allClustersSizeSum, "The total sum of customers is not correct on the travel");
            }

            // Total distance is correct
            Debug.Assert(Math.Truncate(totalCustomerRouteDistance) == Math.Truncate(Functions.calculateCustomerTotalTravelDistanceForVehicle(customersWeakRoute, instance.customersDistanceMatrix)), "CustomerWeak - The final distance is not correct");

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

        // Write a python script to draw the solution
        public static void solutionDrawPythonCode(CluVRPInstance instance, CluVRPSolution solution)
        {
            // Lines of file
            List<string> lines = new List<string>();

            // Add initial lines to script
            lines.Add("import matplotlib.pyplot as plt");
            lines.Add("plt.figure(num=None, figsize=(24, 16), dpi=120, facecolor='w', edgecolor='k')");

            // For python vars
            string[] abc = new string[]{ "a", "b", "c", "d", "f", "g", "h", "i", "j", "k", "l", "m", "k", "r", "t", "u", "v", "w", "y", "z",
            "aa", "ab", "ac", "ad", "af", "ag", "ah", "ai", "aj", "ak", "al", "am", "ak", "ar", "at", "au", "av", "aw", "ay", "az",
            "ca", "cb", "cc", "cd", "cf", "cg", "ch", "ci", "cj", "ck", "cl", "cm", "ck", "cr", "ct", "cu", "cv", "cw", "cy", "cz"};

            // Array of colors
            string[] colors = new string[] { "aqua", "bisque", "black", "blanchedalmond", "blue", "blueviolet", "brown", "burlywood", "cadetblue", "chartreuse", "chocolate", "coral", "cornflowerblue", "cornsilk", "crimson", "cyan", "darkblue", "darkcyan", "darkgoldenrod", "darkgray", "darkgreen", "darkgrey", "darkkhaki", "darkmagenta", "darkolivegreen", "darkorange", "darkorchid", "darkred", "darksalmon", "darkseagreen", "darkslateblue", "darkslategray", "darkslategrey", "darkturquoise", "darkviolet", "deeppink", "deepskyblue", "dimgray", "dimgrey", "dodgerblue", "firebrick", "floralwhite", "forestgreen", "fuchsia", "gainsboro", "ghostwhite", "gold", "goldenrod", "gray", "green", "greenyellow", "grey", "honeydew", "hotpink", "indianred", "indigo", "ivory", "khaki", "lavender", "lavenderblush", "lawngreen", "lemonchiffon", "lightblue", "lightcoral", "lightcyan", "lightgoldenrodyellow", "lightgray", "lightgreen", "lightgrey", "lightpink", "lightsalmon", "lightseagreen", "lightskyblue", "lightslategray", "lightslategrey", "lightsteelblue", "lightyellow", "lime", "limegreen", "linen", "magenta", "maroon", "mediumaquamarine", "mediumblue", "mediumorchid", "mediumpurple", "mediumseagreen", "mediumslateblue", "mediumspringgreen", "mediumturquoise", "mediumvioletred", "midnightblue", "mintcream", "mistyrose", "moccasin", "navajowhite", "navy", "oldlace", "olive", "olivedrab", "orange", "orangered", "orchid", "palegoldenrod", "palegreen", "paleturquoise", "palevioletred", "papayawhip", "peachpuff", "peru", "pink", "plum", "powderblue", "purple", "rebeccapurple", "red", "rosybrown", "royalblue", "saddlebrown", "salmon", "sandybrown", "seagreen", "seashell", "sienna", "silver", "skyblue", "slateblue", "slategray", "slategrey", "snow", "springgreen", "steelblue", "tan", "teal", "thistle", "tomato", "turquoise", "violet", "wheat", "white", "whitesmoke", "yellow", "yellowgreen" };
            string[] vehicle_colors = new string[] { "red", "green", "blue", "orange", "purple", "cyan", "magenta", "lime", "pink", "teal", "lavender", "brown", "beige", "maroon", "mint", "olive", "coral", "navy", "grey", "white", "black" };

            // Variables
            int colorIt = 0;
            int var1 = 0;
            int var2 = 1;
            string outFileName = instance.file_name + ".py";

            // Code for draw the customers points
            // All customers of one cluster have the same color
            string array1 = abc[var1] + "= [" + instance.nodes[0].x.ToString().Replace(',', '.') + "]";
            string array2 = abc[var2] + "= [" + instance.nodes[0].y.ToString().Replace(',', '.') + "]";
            string array3 = "plt.plot(" + abc[0] + "," + abc[1] + ",'ro', markersize= 44, color = '" + colors[0] + "')";
            lines.Add(array1);
            lines.Add(array2);
            lines.Add(array3);
            var1 = var1 + 2;
            var2 = var2 + 2;
            colorIt++;
            for (int clusterIt = 1; clusterIt < instance.clusters.Length; clusterIt++)
            {                
                array1 = abc[var1] + "= [";
                array2 = abc[var2] + "= [";

                for (int customerIt = 0; customerIt  < instance.clusters[clusterIt].Length; customerIt++)
                {
                    int customer = instance.clusters[clusterIt][customerIt] - 1;
                    array1 += instance.nodes[customer].x.ToString().Replace(',', '.') + ",";
                    array2 += instance.nodes[customer].y.ToString().Replace(',', '.') + ",";
                }

                StringBuilder sb = new StringBuilder(array1);
                sb[sb.Length - 1] = ']';
                array1 = sb.ToString();
                sb = new StringBuilder(array2);
                sb[sb.Length - 1] = ']';
                array2 = sb.ToString();
                array3 = "plt.plot(" + abc[var1] + "," + abc[var2] + ",'ro', color = '" + colors[colorIt] + "')";

                // For centroides
                var1 = (var1 + 2) % abc.Length;
                var2 = (var1 + 2) % abc.Length;
                string array4 = abc[var1] + "= [";
                string array5 = abc[var2] + "= [";
                array4 += instance.clustersCentroid[clusterIt].Item1.ToString().Replace(',', '.') + "]";
                array5 += instance.clustersCentroid[clusterIt].Item2.ToString().Replace(',', '.') + "]";
                string array6 = "plt.plot(" + abc[var1] + "," + abc[var2] + ",'ro',markersize=22, color = '" + colors[colorIt] + "')";
                
                // Write python code for clusters and customers
                lines.Add(array1);
                lines.Add(array2);
                lines.Add(array3);
                lines.Add(array4);
                lines.Add(array5);
                lines.Add(array6);

                // Change color and variables name for next cluster
                var1 = (var1 + 2) % abc.Length;
                var2 = (var2 + 2) % abc.Length;
                colorIt = (colorIt + 1) % colors.Length;
            }

            // Draw vehicle travel
            var1 = 0;
            var2 = 1; 
            colorIt = 0;
            for(int vehicle = 0; vehicle < solution.customersPaths.Length; vehicle++)
            {
                array1 = abc[var1] + "= [";
                array2 = abc[var2] + "= [";
                for(int clusterIt = 0; clusterIt < solution.customersPaths[vehicle].Length; clusterIt++)
                {
                    for(int customerIt = 0; customerIt < solution.customersPaths[vehicle][clusterIt].Count; customerIt++)
                    {
                        int customer = solution.customersPaths[vehicle][clusterIt][customerIt] - 1;
                        array1 += instance.nodes[customer].x.ToString().Replace(',', '.') + ",";
                        array2 += instance.nodes[customer].y.ToString().Replace(',', '.') + ",";
                    }
                }
                StringBuilder sb = new StringBuilder(array1);
                sb[sb.Length-1] = ']';
                array1 = sb.ToString();
                sb = new StringBuilder(array2);
                sb[sb.Length - 1] = ']';
                array2 = sb.ToString();
                
                array3 = "plt.plot(" + abc[var1] + "," + abc[var2] + ", color = '" + vehicle_colors[colorIt] + "')";

                // Write python code for clusters and customers
                lines.Add(array1);
                lines.Add(array2);
                lines.Add(array3);
                
                // Change color and variables name for next cluster
                var1 = (var1 + 2) % abc.Length;
                var2 = (var2 + 2) % abc.Length;
                colorIt = (colorIt + 1) % vehicle_colors.Length;
            }

            // Add final line
            lines.Add("plt.savefig('" + instance.file_name + ".png')");

            // Write file
            try
            {
                System.IO.File.WriteAllLines(outFileName, lines);
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }

            // Excute script
            System.Diagnostics.Process.Start(PYTHON_PATH, outFileName);
        }

    }
}
