using System;

namespace CluVRP_GRASP
{
    static class InstanceParser
    {

        static int GVRP_NODE_COORD_SECTION_IDX = 8; 

        static public GVRPInstance[] loadGVRPSetOfInstances(string InstanceSetName)
        {

            string filePath = InstanceSetName + ".set";
            GVRPInstance[] gvrpInstances;

            try
            {
                string[] lines = System.IO.File.ReadAllLines(filePath);
                gvrpInstances = new GVRPInstance[lines.Length];

                foreach (string instanceFilePath in lines)
                {
                    int i = 0;
                    try
                    {
                        GVRPInstance instance;
                        string[] instanceFileText = System.IO.File.ReadAllLines(instanceFilePath);
                        instance = parseGVRPInstance(instanceFileText);
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

            return gvrpInstances;
        }

        static private GVRPInstance parseGVRPInstance(string[] instanceText)
        {
            // Set static parameters
            string name = instanceText[0].Substring(7);
            string comment = instanceText[1].Substring(10);
            int dimension = Int32.Parse(instanceText[2].Substring(12));
            int vehicules = Int32.Parse(instanceText[3].Substring(11));
            int gvrp_sets = Int32.Parse(instanceText[4].Substring(12));
            int capacity = Int32.Parse(instanceText[5].Substring(11));
            string edge_weight_type = instanceText[6].Substring(19);

            // Set dynamic parameters
            NodePoint[] nodes = new NodePoint[dimension];
            int[][] clusters = new int[gvrp_sets][];
            int[] clusters_demand = new int[gvrp_sets];
            int GVRP_SET_SECTION_IDX = GVRP_NODE_COORD_SECTION_IDX + dimension + 1;
            int GVRP_DEMAND_SECTION_IDX = GVRP_SET_SECTION_IDX + gvrp_sets + 1;

            // Separator for split
            Char[] separator = new Char[] {' '};

            // Build Nodes Array
            for (int i = 0; i < dimension; i++)
            {
                string[] nodeParsed = instanceText[GVRP_NODE_COORD_SECTION_IDX + i].Split(separator);
                int x = Int32.Parse(nodeParsed[1]);
                int y = Int32.Parse(nodeParsed[2]);
                NodePoint node = new NodePoint(x, y);
                nodes[i] = node;
            }

            // Build Clusters Array
            for (int i = 0; i < gvrp_sets; i++)
            {
                string[] setParsed = instanceText[GVRP_SET_SECTION_IDX + i].Split(separator);
                clusters[i] = new int[setParsed.Length - 2];
                int idx = 0;
                for (int j = 1; j + 1 < setParsed.Length; j++)
                {
                    clusters[i][idx] = Int32.Parse(setParsed[j]);
                    idx++;
                }               
            }

            // Build Custumer demand Array
            for (int i = 0; i < gvrp_sets; i++)
            {
                string[] demandParsed = instanceText[GVRP_DEMAND_SECTION_IDX + i].Split(separator);
                clusters_demand[i] = Int32.Parse(demandParsed[1]);
             }

            return new GVRPInstance(name, comment, dimension, vehicules, gvrp_sets, capacity, edge_weight_type,
                nodes, clusters, clusters_demand);
        }
        
    }
}
