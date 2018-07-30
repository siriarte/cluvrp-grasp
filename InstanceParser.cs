using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace cluvrp_grasp
{
    enum Instance {GVRP, GoldenBattarra, GoldenIzquierdo}

    static class InstanceParser
    {
        // Static values to read GVRP instances
        static int GVRP_NODE_COORD_SECTION_IDX = 8;
        static int GVRP_NODE_COORD_SECTION_IDX_GOLDEN = 7;
        static int GVRP_NODE_COORD_SECTION_IDX_GOLDEN_IZQUIERDO = 8;
        static bool NODES_TO_FILE = true;

        /*
         * 
         * Read every instance file of InstanceSetName file and create a instance of it
         * 
         */
        static public CluVRPInstance[] loadGVRPSetOfInstances(string InstanceSetName)
        {

            // Init variables
            string filePath = InstanceSetName;
            CluVRPInstance[] gvrpInstances;

            // Main Cycle
            try
            {
                string[] lines = System.IO.File.ReadAllLines(filePath);
                gvrpInstances = new CluVRPInstance[lines.Length];

                // Get all the instances setted on the file
                int i = 0;
                foreach (string instanceFilePath in lines)
                {
                    // Try to parse instance
                    try
                    {
                        CluVRPInstance instance;
                        string[] instanceFileText = System.IO.File.ReadAllLines(instanceFilePath);
                        if (instanceFilePath.Contains("rho"))
                        {
                            instance = parseGoldenIzquierdoInstance(instanceFilePath, instanceFileText);

                        }
                        else if (instanceFilePath.Contains("Golden"))
                        {
                            instance = parseGoldenBattarraInstance(instanceFilePath, instanceFileText);

                        } else {
                            instance = parseGVRPInstance(instanceFilePath, instanceFileText);
                        }
                        gvrpInstances[i] = instance;
                        i++;
                    }
                    catch (Exception e)
                    {
                        Logger.GetInstance().logLine(e.ToString());
                    }
                }

                
            }catch(Exception e)
            {
                Logger.GetInstance().logLine(e.ToString());
                return null;
            }

            // Return array of instances
            return gvrpInstances;
        }

        /*
         * Parser for GVRP02 and GVRP03 modified instances. 
         * http://www.personal.soton.ac.uk/tb12v07/gvrp.html
         * Instances of CVRP (Bektas 2011) a adapted by Battarra (2014)  
         */
        static private CluVRPInstance parseGVRPInstance(string fileName, string[] instanceText)
        {
            // Set static parameters
            string file_name = fileName;
            string name = instanceText[0].Substring(7);
            string comment = instanceText[1].Substring(10);
            int dimension = Int32.Parse(instanceText[2].Substring(12)); 
            int vehicules = Int32.Parse(instanceText[3].Substring(11));
            int gvrp_sets = Int32.Parse(instanceText[4].Substring(12));
            int capacity = Int32.Parse(instanceText[5].Substring(11));
            string edge_weight_type = instanceText[6].Substring(19);
            int depot = 1;

            // Set dynamic parameters
            NodePoint[] nodes = new NodePoint[dimension]; //sum 1 for the start node 
            int[][] clusters = new int[gvrp_sets + 1][];
            int[] clusters_demand = new int[gvrp_sets + 1];
            int GVRP_SET_SECTION_IDX = GVRP_NODE_COORD_SECTION_IDX + dimension + 1;
            int GVRP_DEMAND_SECTION_IDX = GVRP_SET_SECTION_IDX + gvrp_sets + 1;

            // Separator for split
            Char[] separator = new Char[] {' '};

            // Build Nodes Array
            //nodes[0] = new NodePoint(0, 0); // For the depot
            for (int i = 0; i < nodes.Length; i++)
            {
                string[] nodeParsed = instanceText[GVRP_NODE_COORD_SECTION_IDX + i].Split(separator);
                int x = Int32.Parse(nodeParsed[1]);
                int y = Int32.Parse(nodeParsed[2]);
                NodePoint node = new NodePoint(x, y);
                nodes[i] = node;
            }

            // Build Clusters Array
            clusters[0] = new int[1];
            clusters[0][0] = depot;
            for (int i = 0; i < gvrp_sets; i++)
            {
                string[] setParsed = instanceText[GVRP_SET_SECTION_IDX + i].Split(separator);
                clusters[i+1] = new int[setParsed.Length - 2];
                int idx = 0;
                for (int j = 1; j + 1 < setParsed.Length; j++)
                {
                    clusters[i+1][idx] = Int32.Parse(setParsed[j]);
                    idx++;
                }               
            }

            // Build Custumer demand Array
            clusters_demand[0] = 0;
            for (int i = 0; i < gvrp_sets; i++)
            {
                string[] demandParsed = instanceText[GVRP_DEMAND_SECTION_IDX + i].Split(separator);
                clusters_demand[i+1] = Int32.Parse(demandParsed[1]);
             }

            // Output points instance file
            if (NODES_TO_FILE)
            {
                pointsToFile(clusters, nodes, fileName);
            }

            // Return parsed instance
            return new CluVRPInstance(Instance.GVRP, file_name, name, comment, dimension, vehicules, gvrp_sets, capacity, edge_weight_type,
                nodes, clusters, clusters_demand, depot);
        }

        /*
         * Parser for Golden instances. 
         *  Instances of CVRP (Bektas 2011) a adapter by Battarra (2014)  
         */
        static private CluVRPInstance parseGoldenBattarraInstance(string fileName, string[] instanceText)
        {
            // Set static parameters
            string file_name = fileName;
            string name = instanceText[0].Substring(6);
            string comment = instanceText[1].Substring(9);
            int dimension = Int32.Parse(instanceText[2].Substring(12));
            int vehicules = Int32.Parse(instanceText[3].Substring(11));
            int gvrp_sets = Int32.Parse(instanceText[4].Substring(12));
            int capacity = Int32.Parse(instanceText[5].Substring(11));
            string edge_weight_type = instanceText[6].Substring(19);
            int depot = 1;

            // Set dynamic parameters
            NodePoint[] nodes = new NodePoint[dimension]; //sum 1 for the start node 
            int[][] clusters = new int[gvrp_sets + 1][];
            int[] clusters_demand = new int[gvrp_sets + 1];
            int GVRP_SET_SECTION_IDX = GVRP_NODE_COORD_SECTION_IDX + dimension;
            int GVRP_DEMAND_SECTION_IDX = GVRP_SET_SECTION_IDX + gvrp_sets + 1;

            // Separator for split
            String[]separator = new String[] { "  " };

            // Build Nodes Array
            //nodes[0] = new NodePoint(0, 0); // For the depot
            for (int i = 0; i < nodes.Length; i++)
            {
                string[] nodeParsed = instanceText[GVRP_NODE_COORD_SECTION_IDX_GOLDEN + i].Split(' ');
                double x = Double.Parse(nodeParsed[1], CultureInfo.InvariantCulture);
                double y = Double.Parse(nodeParsed[2], CultureInfo.InvariantCulture);
                NodePoint node = new NodePoint(x, y);
                nodes[i] = node;
            }

            // Build Clusters Array
            clusters[0] = new int[1];
            clusters[0][0] = depot;
            for (int i = 0; i < gvrp_sets; i++)
            {
                string lineToParse = instanceText[GVRP_SET_SECTION_IDX + i].Substring(3);
                lineToParse = lineToParse.Substring(0, lineToParse.Length - 3);
                string[] setParsed = lineToParse.Split(separator, StringSplitOptions.RemoveEmptyEntries);
                clusters[i + 1] = new int[setParsed.Length];
                int idx = 0;
                for (int j = 0; j < setParsed.Length; j++)
                {
                    clusters[i + 1][idx] = Int32.Parse(setParsed[j]);
                    idx++;
                }
            }

            // Build Custumer demand Array
            clusters_demand[0] = 0;
            for (int i = 0; i < gvrp_sets; i++)
            {
                string[] demandParsed = instanceText[GVRP_DEMAND_SECTION_IDX + i].Split(new String[] { " " }, StringSplitOptions.RemoveEmptyEntries);
                clusters_demand[i + 1] = Int32.Parse(demandParsed[1]);
            }

            // Output points instance file
            if (NODES_TO_FILE)
            {
                pointsToFile(clusters, nodes, fileName);
            }

            // Return parsed instance
            return new CluVRPInstance(Instance.GoldenBattarra, file_name, name, comment, dimension, vehicules, gvrp_sets, capacity, edge_weight_type,
                nodes, clusters, clusters_demand, depot);
        }

        /*
         * Parser for Golden instances. 
         * Instances of CVRP (Bektas 2011) adapted by Exposito-Izquierdo
         */
        static private CluVRPInstance parseGoldenIzquierdoInstance(string fileName, string[] instanceText)
        {
            // Set static parameters
            string file_name = fileName;
            int dimension = Int32.Parse(instanceText[3].Substring(12));
            int capacity = Int32.Parse(instanceText[4].Substring(11));
            string edge_weight_type = instanceText[5].Substring(19);
            int depot = 1;

            // Set dynamic parameters
            NodePoint[] nodes = new NodePoint[dimension];
            int[] cluster_by_nodes = new int[dimension];
            int[] demand = new int[dimension];
            int GVRP_DEMAND_SECTION_IDX = GVRP_NODE_COORD_SECTION_IDX_GOLDEN_IZQUIERDO + dimension + 1;
            int GVRP_CLUSTER_SECTION_IDX = GVRP_DEMAND_SECTION_IDX + dimension + 1;

            // Separator for split
            String[] separator = new String[] { "  " };

            // Build Nodes Array
            //nodes[0] = new NodePoint(0, 0); // For the depot
            for (int i = 0; i < nodes.Length; i++)
            {
                string[] nodeParsed = instanceText[GVRP_NODE_COORD_SECTION_IDX_GOLDEN_IZQUIERDO + i].Split(' ');
                double x = Double.Parse(nodeParsed[1], CultureInfo.InvariantCulture);
                double y = Double.Parse(nodeParsed[2], CultureInfo.InvariantCulture);
                NodePoint node = new NodePoint(x, y);
                nodes[i] = node;
            }

            // Build Demand Array
            for (int i = 0; i < demand.Length; i++)
            {
                string lineToParse = instanceText[GVRP_DEMAND_SECTION_IDX + i];
                string[] setParsed = instanceText[GVRP_DEMAND_SECTION_IDX + i].Split(' ');
                demand[i] = int.Parse(setParsed[1]);
            }

            // Build cluster Array
            for (int i = 1; i < cluster_by_nodes.Length; i++)
            {
                string lineToParse = instanceText[GVRP_CLUSTER_SECTION_IDX + i];
                string[] setParsed = instanceText[GVRP_CLUSTER_SECTION_IDX + i].Split(' ');
                cluster_by_nodes[i] = int.Parse(setParsed[1]);
            }

            // Transform to neccesary cluster array struct
            Dictionary <int, List<int>> clusters_dict = new Dictionary < int, List< int >>();
            clusters_dict[0] = new List < int >{ 1 };
            for(int i = 1; i < cluster_by_nodes.Length; i++)
            {
                int cluster = cluster_by_nodes[i];
                if (!clusters_dict.ContainsKey(cluster))
                {
                    clusters_dict[cluster] = new List<int>();
                }

                clusters_dict[cluster].Add(i + 1);
            }

            int[][] clusters = new int[clusters_dict.Keys.Count][]; 
            foreach (int cluster in clusters_dict.Keys)
            {
                List<int> nodesInClusters = clusters_dict[cluster];
                clusters[cluster] = new int[nodesInClusters.Count];
                for(int j = 0; j < clusters[cluster].Length; j++)
                {
                    clusters[cluster][j] = nodesInClusters[j];
                }
            }

            // Transform to neccesary demand array struct
            int[] clusters_demand = new int[clusters.Length];
            for(int i = 0; i < demand.Length; i++)
            {
                foreach (int cluster in clusters_dict.Keys)
                {
                    if (clusters_dict[cluster].Contains(i + 1))
                    {
                        clusters_demand[cluster] += demand[i];
                        break;
                    }
                }
            }

            // For this instance the vehicle number is free
            int vehicles = clusters.Length - 1;

            // Output points instance file
            if (NODES_TO_FILE)
            {
                pointsToFile(clusters, nodes, fileName);
            }

            // Return parsed instance
            return new CluVRPInstance(Instance.GoldenIzquierdo, file_name, "", "", dimension, vehicles, clusters_demand.Length, capacity, edge_weight_type,
                nodes, clusters, clusters_demand, depot);            
        }

        static public void pointsToFile(int[][] clusters, NodePoint[] nodes, string fileName)
        {

            string[] abc = new string[]{ "a", "b", "c", "d", "f", "g", "h", "i", "j", "k", "l", "m", "k", "r", "t", "u", "v", "w", "y", "z",
            "aa", "ab", "ac", "ad", "af", "ag", "ah", "ai", "aj", "ak", "al", "am", "ak", "ar", "at", "au", "av", "aw", "ay", "az",
            "ca", "cb", "cc", "cd", "cf", "cg", "ch", "ci", "cj", "ck", "cl", "cm", "ck", "cr", "ct", "cu", "cv", "cw", "cy", "cz"};

            string[] colors = new string[] {"aliceblue", "antiquewhite", "aqua", "aquamarine", "azure", "beige", "bisque", "black", "blanchedalmond", "blue", "blueviolet", "brown", "burlywood", "cadetblue", "chartreuse", "chocolate", "coral", "cornflowerblue", "cornsilk", "crimson", "cyan", "darkblue", "darkcyan", "darkgoldenrod", "darkgray", "darkgreen", "darkgrey", "darkkhaki", "darkmagenta", "darkolivegreen", "darkorange", "darkorchid", "darkred", "darksalmon", "darkseagreen", "darkslateblue", "darkslategray", "darkslategrey", "darkturquoise", "darkviolet", "deeppink", "deepskyblue", "dimgray", "dimgrey", "dodgerblue", "firebrick", "floralwhite", "forestgreen", "fuchsia", "gainsboro", "ghostwhite", "gold", "goldenrod", "gray", "green", "greenyellow", "grey", "honeydew", "hotpink", "indianred", "indigo", "ivory", "khaki", "lavender", "lavenderblush", "lawngreen", "lemonchiffon", "lightblue", "lightcoral", "lightcyan", "lightgoldenrodyellow", "lightgray", "lightgreen", "lightgrey", "lightpink", "lightsalmon", "lightseagreen", "lightskyblue", "lightslategray", "lightslategrey", "lightsteelblue", "lightyellow", "lime", "limegreen", "linen", "magenta", "maroon", "mediumaquamarine", "mediumblue", "mediumorchid", "mediumpurple", "mediumseagreen", "mediumslateblue", "mediumspringgreen", "mediumturquoise", "mediumvioletred", "midnightblue", "mintcream", "mistyrose", "moccasin", "navajowhite", "navy", "oldlace", "olive", "olivedrab", "orange", "orangered", "orchid", "palegoldenrod", "palegreen", "paleturquoise", "palevioletred", "papayawhip", "peachpuff", "peru", "pink", "plum", "powderblue", "purple", "rebeccapurple", "red", "rosybrown", "royalblue", "saddlebrown", "salmon", "sandybrown", "seagreen", "seashell", "sienna", "silver", "skyblue", "slateblue", "slategray", "slategrey", "snow", "springgreen", "steelblue", "tan", "teal", "thistle", "tomato", "turquoise", "violet", "wheat", "white", "whitesmoke", "yellow", "yellowgreen"};

            int color = 0;
            int var1 = 0;
            int var2 = 1;
            fileName = fileName + ".points";

            for (int clusterIt = 0; clusterIt < clusters.Length; clusterIt++)
            {
                string array1 = abc[var1] + "= [";
                string array2 = abc[var2] + "= [";

                for (int customerIt = 0; customerIt + 1 < clusters[clusterIt].Length; customerIt++)
                {
                    int customer = clusters[clusterIt][customerIt] - 1; 
                    array1 += nodes[customer].x.ToString().Replace(',', '.') + ",";
                    array2 += nodes[customer].y.ToString().Replace(',', '.') + ",";
                }
                int lastCustomer = clusters[clusterIt][clusters[clusterIt].Length - 1] - 1;
                array1 += nodes[lastCustomer].x.ToString().Replace(',', '.') + "]";
                array2 += nodes[lastCustomer].y.ToString().Replace(',', '.') + "]";

                string array3 = "plt.plot(" + abc[var1] + "," + abc[var2] + ",'ro', color = '"+ colors[color] + "')";

                try
                {
                    using (System.IO.StreamWriter file = System.IO.File.AppendText(fileName))
                    {
                        file.WriteLine(array1);
                        file.WriteLine(array2);
                        file.WriteLine(array3);
                    }
                }
                catch (Exception e)
                {
                    Console.WriteLine(e.ToString());
                }

                var1 = (var1 + 2) % abc.Length;
                var2 = (var2 + 2) % abc.Length;
                color = (color + 1) % colors.Length;
            }
        }
    }
}
