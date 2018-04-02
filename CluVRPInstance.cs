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
        private string _file_name;
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

        public CluVRPInstance()
        {

        }

        public CluVRPInstance(string file_name, string name, string comment, int dimension, int vehicules, int gvrp_sets, 
            int capacity, string edge_weight_type, NodePoint[] nodes, int[][] clusters, int[] clusters_demand)
        {
            this._file_name = file_name;
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

        public string file_name() { return this._file_name; }
        public string name() { return this._name; }
        public string comment() { return this._comment; }
        public int dimension() { return this._dimension; }
        public int vehicules() { return this._vehicules; }
        public int gvrp_sets() { return this._gvrp_sets; }
        public int capacity() { return this._capacity; }
        public string edge_weight_type() { return this._edge_weight_type; }
        public NodePoint[] nodes() { return this._nodes; }
        public int[][] clusters() { return this._clusters; }
        public int[] clusters_demand() { return this._clusters_demand; }

    }

}