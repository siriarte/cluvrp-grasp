using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CluVRP_GRASP
{

    class NodePoint
    {
        private int x;
        private int y;

        public NodePoint(int x, int y)
        {
            this.x = x;
            this.y = y;
        }

        public int getX() { return x; }
        public int getY() { return y; }
    }

    class CluVRPInstance
    {

    }

    class GVRPInstance : CluVRPInstance
    {
        private string _name;
        private string _comment;
        private int _dimension;
        private int _vehicules;
        private int _gvrp_sets;
        private int _capacity;
        private string _edge_weight_type;
        private NodePoint[] _nodes;
        private int[][] _clusters;
        private int[] _clusters_demand;

        public GVRPInstance()
        {

        }

        public GVRPInstance(string name, string comment, int dimension, int vehicules, int gvrp_sets, 
            int capacity, string edge_weight_type, NodePoint[] nodes, int[][] clusters, int[] clusters_demand)
        {
            this._name = name;
            this._comment = comment;
            this._dimension = dimension;
            this._vehicules = vehicules;
            this._gvrp_sets = gvrp_sets;
            this._capacity = capacity;
            this._edge_weight_type = edge_weight_type;
            this._nodes = nodes;
            this._clusters = clusters;
            this._clusters_demand = clusters_demand;
        }

        public string name { get; }
        public string comment { get; }
        public int dimension { get; }
        public int vehicules { get; }
        public int gvrp_sets { get; }
        public int capacity { get; }
        public int edge_weight_type { get; }
        public NodePoint[] nodes { get; }
        public int clusters { get; }
        public int clusters_demand { get; }

    }

}