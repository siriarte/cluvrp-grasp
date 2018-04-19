using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;

namespace cluvrp_grasp
{
    class CustomerSolution
    {
        private double[] _vehiculeRouteDistance;
        public List<int>[][] _customersCircuit { set; get; }
        public double _totalRouteDistance { set; get; }
        public ClusterSolution _clusterSolution { set; get; }

        // Constructor 
        public CustomerSolution(ClusterSolution clusterSolution, int numberOfVehicules)
        {
            this._customersCircuit = null;
            this._clusterSolution = clusterSolution;
            this._vehiculeRouteDistance = new double[numberOfVehicules];
            Functions.Populate(this.vehiculeRouteDistance, double.MaxValue);
            this._totalRouteDistance = _vehiculeRouteDistance.Sum();
        }
      
        // Seter
        public double[] vehiculeRouteDistance
        {
            get { return this._vehiculeRouteDistance; }
            set
            {
                _vehiculeRouteDistance = value;
                _totalRouteDistance = vehiculeRouteDistance.Sum();

            }
        }
        
        // Print solution information
        public void printSolution()
        {
           string tittle = "RESULT:";
           string clusterDistance = _clusterSolution.totalRouteDistance.ToString();
           string totalDistance = _totalRouteDistance.ToString();
           string vehiculeDistance = string.Join(" -- ", _vehiculeRouteDistance);
           
           Logger.GetInstance().logLine(tittle);
           Logger.GetInstance().logLine(clusterDistance);
           Logger.GetInstance().logLine(totalDistance);
           Logger.GetInstance().logLine(vehiculeDistance);
                        
           for(int i = 0; i < _customersCircuit.Length; i++)
           {
                string vehiculeTittle = "Route for vehicle: " + i;
                Logger.GetInstance().logLine(vehiculeTittle);

                for (int clusterIt = 0; clusterIt < _customersCircuit[i].Length; clusterIt++)
                {
                    string circuit = string.Join(", ", _customersCircuit[i][clusterIt]);
                    Logger.GetInstance().logLine(circuit);
                }
            }    

        }

        // Verify the solution is correct respect to the instance
        public void verifySolution(CluVRPInstance instance, double[][] customersDistanceMatrix)
        {
            // Verify number of vehicles
            int numberOfVehicles = instance.vehicles();
            Debug.Assert(_customersCircuit.Length == numberOfVehicles);
            Debug.Assert(_vehiculeRouteDistance.Length == numberOfVehicles);
            
            // Verify number of clusters
            int numberOfClusters = instance.clusters().Length;
            int clustersCounter = 0;
            for(int i = 0; i < _customersCircuit.Length; i++)
            {
                clustersCounter += _customersCircuit[i].Length;
            }
            Debug.Assert(numberOfClusters == clustersCounter - (_customersCircuit.Length * 2) + 1);

            // Verify number of customers
            int numberOfCustomers = instance.dimension();
            int customersCounter = 0;
            for (int i = 0; i < _customersCircuit.Length; i++)
            {
                for (int j = 0; j < _customersCircuit[i].Length; j++)
                {
                    customersCounter += _customersCircuit[i][j].Count;
                }
            }
            Debug.Assert(numberOfCustomers == customersCounter - (numberOfVehicles * 2) + 1);

            // All clusters are correct
            List<int>[] vehicleRoute = _clusterSolution.clusterRouteForVehicule;
            for(int vehicle = 0; vehicle < vehicleRoute.Length; vehicle++)
            {
                List<int> clusterList = vehicleRoute[vehicle];
                for(int clusterIt = 0; clusterIt < clusterList.Count; clusterIt++)
                {
                    int clusterNumber = clusterList[clusterIt];
                    List<int> cluster = _customersCircuit[vehicle][clusterIt].ToList<int>();
                    List<int> clusterInstance = instance.clusters()[clusterNumber].ToList<int>();
                    bool containsAll = Functions.ContainsAllItems(cluster, clusterInstance);
                    Debug.Assert(containsAll && cluster.Count == clusterInstance.Count);
                }
            }

            // Total distance is correct
            Debug.Assert(_totalRouteDistance == Functions.calculateTotalTravelDistance(_customersCircuit, customersDistanceMatrix));

        }

    }
}
