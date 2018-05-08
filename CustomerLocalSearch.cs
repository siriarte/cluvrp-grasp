using System;
using System.Collections.Generic;
using System.Linq;

namespace cluvrp_grasp
{

    class CustomerLocalSearch
    {
        // Attributes
        public int maxIterationsWithoutImprovementTwoOpt { get; set; }
        public int maxIterationsWithoutImprovementRelocate { get; set; }
        public int maxIterationsWithoutImprovementExchange { get; set; }
        public CluVRPSolution solution { get; set; }
        public CluVRPInstance instance { get; set; }

        // Constructor
        public CustomerLocalSearch(CluVRPSolution solution, 
            CluVRPInstance instance,
            int maxIterationsWithoutImprovementTwoOpt = 100,
            int maxIterationsWithoutImprovementRelocate = 100,
            int maxIterationsWithoutImprovementExchange = 100)
        {
            this.solution = solution;
            this.instance = instance;
            this.maxIterationsWithoutImprovementTwoOpt = maxIterationsWithoutImprovementTwoOpt;
            this.maxIterationsWithoutImprovementRelocate = maxIterationsWithoutImprovementRelocate;
            this.maxIterationsWithoutImprovementExchange = maxIterationsWithoutImprovementExchange;
        }

        // TwoOpt local-search 
        public void twoOpt()
        {
            // Init variables
            List<int>[][] customersCircuit = solution.customersPaths;
            int numberOfVehicles = customersCircuit.Length;
          
            // For each vehicle route
            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                // Calculate the complete travel distance by vehicle
                List<int>[] clusterRoute = customersCircuit[vehicle];
          
                // Main cycle
                int iteration = 0;
                while (iteration < maxIterationsWithoutImprovementTwoOpt)
                {
                    // For every cluster on the route
                    for (int clusterIt = 0; clusterIt < clusterRoute.Length; clusterIt++)
                    {
                        // For every customer path of a cluster
                        List<int> route = clusterRoute[clusterIt];
                        int maxW = route.Count();

                        // Starting from second customer (avoid depot)
                        for (int i = 1; i + 1< maxW; i++)
                        {
                            // Until the last one on the path
                            for (int j = i + 1; j < maxW; j++)
                            {
                                // Backup original route
                                List<int> oldRoute = customersCircuit[vehicle][clusterIt];

                                // New route create by two-opt-swap between customer i and j 
                                List<int> newRoute = twoOptSwap(customersCircuit[vehicle][clusterIt], i, j);

                                // Assign the new route to the circuit
                                customersCircuit[vehicle][clusterIt] = newRoute;

                                // Calculate the new distance
                                double newDistance = Functions.calculateTotalTravelDistance(customersCircuit, instance.customersDistanceMatrix, vehicle);

                                // If distance if better
                                if (newDistance + 0.5 < this.solution.vehiculeRouteDistance[vehicle])
                                {
                                    // Update best distance
                                    this.solution.vehiculeRouteDistance[vehicle] = newDistance;
                                    
                                    // Restart iterator
                                    iteration = 0;
                                }
                                else
                                {
                                    // Restore old route to the circuit
                                    customersCircuit[vehicle][clusterIt] = oldRoute;
                                }
                            } // end for j
                        } // end for i

                        // Increase iterator
                        iteration++;

                    } // End for clusterIt
                } // End for iterator
            } // End for vehicle

            // Update the total customer route distance (the sum all vehicle route distances)
            solution.totalCustomerRouteDistance = this.solution.vehiculeRouteDistance.Sum();
        }
        
        // TwoOpt Algorithm
        public List<int> twoOptSwap(List<int> route, int i, int k)
        {
            List<int> newRoute = route.GetRange(0, i);
            int reverseSize = k - i + 1;
            List<int> reverseRoute = route.GetRange(i, reverseSize);
            reverseRoute.Reverse();
            int restSize = route.Count - (k + 1);
            List<int> endRoute = route.GetRange(k + 1, restSize);
            newRoute.AddRange(reverseRoute);
            newRoute.AddRange(endRoute);
            return newRoute;
        }

        // Relocate local-search
        public void relocate()
        {
            // Init variables
            int numberOfVehicles = solution.customersPaths.Length;
            
            // For each vehicle
            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                // Get number of cluster on the vehicle route 
                int numberOfClusters = solution.customersPaths[vehicle].Length;

                // For each cluster
                for (int clusterIt = 0; clusterIt < numberOfClusters; clusterIt++)
                {                    
                    // Get the customer route of the cluste
                    List<int> route = solution.customersPaths[vehicle][clusterIt];

                    // Main cycle
                    int iteration = 0;
                    while (iteration < maxIterationsWithoutImprovementRelocate)
                    {
                        // For each i-customer
                        for (int i = 0; i < route.Count; i++)
                        {
                            // For each j-customer
                            for (int j = 0; j < route.Count; j++)
                            {
                                // If is the same customer or the depot dont do anything
                                if ((i == j) || ((i == 0 && j == route.Count - 1) ||
                                    (i == route.Count - 1 && j == 0)))
                                {
                                    continue;
                                }

                                // If perform realocate success 
                                if (relocate(vehicle, clusterIt, i, j))
                                {
                                    // Restart iterator
                                    iteration = 0;
                                }
                            } // End for j
                        } // End for i

                        // Increase iterator
                        iteration++;

                    } // End for While 
                } // End for clusterIt
            } // End for vehicle
        }

        // Relocate Algorithm
        public bool relocate(int vehicle, int cluster, int i, int j)
        {
            // Get customer path for vehicle and cluster
            List<int> route = solution.customersPaths[vehicle][cluster];

            // Backup the original route
            List<int> oldRoute = new List<int>(route);

            // Perform relocate
            int customer = route[i];
            route.RemoveAt(i);
            route.Insert(j, customer);

            // Set the new route to the path
            solution.customersPaths[vehicle][cluster] = route;

            // Calculate new distance
            double newDistance = Functions.calculateTotalTravelDistance(solution.customersPaths, instance.customersDistanceMatrix, vehicle);

            // If distance was improved
            if (newDistance + 0.5 < solution.vehiculeRouteDistance[vehicle])
            {
                // Update new distance to the vehicle route
                solution.vehiculeRouteDistance[vehicle] = newDistance;
                solution.totalCustomerRouteDistance = solution.vehiculeRouteDistance.Sum();
                return true;
            }
             
            // Else back to the original route
            solution.customersPaths[vehicle][cluster] = oldRoute;
            return false;
        }
        
        /*
        public void exchange()
        {
            int numberOfVehicles = _customerSolution.clusterRouteForVehicule.Length;

            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                List<int> route = _customerSolution.clusterRouteForVehicule[vehicle];
                int iteration = 0;
                while (iteration < _maxIterationsWithoutImprovementExchange)
                {

                    for (int i = 0; i < route.Count; i++)
                    {
                        for (int j = 0; j < route.Count; j++)
                        {
                            if ((i == j) || ((i == 0 && j == route.Count - 1) ||
                                (i == route.Count - 1 && j == 0)))
                                continue;

                            if (exchange(route, vehicle, i, j))
                            {
                                iteration = 0;
                            }
                        }
                    }
                    iteration++;
                }
            }
        }

        bool exchange(List<int> route, int vehicle, int i, int j)
        {

            // Para no irnos del camino
            var celda_anterior_i = i - 1 == -1 ? route.Count - 1 : i - 1;
            var celda_anterior_j = j - 1 == -1 ? route.Count - 1 : j - 1;

            var celda_siguiente_i = i + 1 == route.Count ? 0 : i + 1;
            var celda_siguiente_j = j + 1 == route.Count ? 0 : j + 1;

            // Calculamos el nuevo costo, a ver si es mejor cambiar o no de posición 
            // Costos viejos
            var _distancia_i_izquierda = _customersMatrixDistance[route[celda_anterior_i]][route[i]];
            var _distancia_i_derecha = _customersMatrixDistance[route[i]][route[celda_siguiente_i]];

            var _distancia_j_izquierda = _customersMatrixDistance[route[celda_anterior_j]][route[j]];
            var _distancia_j_derecha = _customersMatrixDistance[route[j]][route[celda_siguiente_j]];

            double nuevo_costo = 0;
            // Costos nuevos
            double _distancia_nueva_i_izquierda, _distancia_nueva_i_derecha, _distancia_nueva_j_izquierda, _distancia_nueva_j_derecha;

            if (i == celda_siguiente_j)
            {
                _distancia_nueva_i_izquierda = _customersMatrixDistance[route[celda_anterior_j]][route[i]];
                _distancia_nueva_i_derecha = _customersMatrixDistance[route[i]][route[j]];

                _distancia_nueva_j_derecha = _customersMatrixDistance[route[j]][route[celda_siguiente_i]];
                _distancia_nueva_j_izquierda = _customersMatrixDistance[route[j]][route[i]];

                //_distancia_nueva_i_derecha = _distancia_nueva_j_izquierda 
                //    = _clusterMatrixDistance[solucion.camino[i], solucion.camino[j]).distancia;
            }
            else if (j == celda_siguiente_i)
            {
                _distancia_nueva_i_derecha = _customersMatrixDistance[route[i]][route[celda_siguiente_j]];
                _distancia_nueva_i_izquierda = _customersMatrixDistance[route[j]][route[i]];

                _distancia_nueva_j_izquierda = _customersMatrixDistance[route[celda_anterior_i]][route[j]];
                _distancia_nueva_j_derecha = _customersMatrixDistance[route[i]][route[j]];

                //_distancia_nueva_i_izquierda = _distancia_nueva_j_derecha = 
                //    _clusterMatrixDistance[solucion.camino[j], solucion.camino[i]).distancia;
            }
            else
            {
                _distancia_nueva_i_izquierda = _customersMatrixDistance[route[celda_anterior_j]][route[i]];
                _distancia_nueva_i_derecha = _customersMatrixDistance[route[i]][route[celda_siguiente_j]];

                _distancia_nueva_j_izquierda = _customersMatrixDistance[route[celda_anterior_i]][route[j]];
                _distancia_nueva_j_derecha = _customersMatrixDistance[route[j]][route[celda_siguiente_i]];
            }

            nuevo_costo = this._customerSolution.totalRouteDistance - _distancia_i_izquierda - _distancia_i_derecha - _distancia_j_izquierda - _distancia_j_derecha +
            _distancia_nueva_i_izquierda + _distancia_nueva_i_derecha + _distancia_nueva_j_izquierda + _distancia_nueva_j_derecha;

            if (nuevo_costo < this._customerSolution.totalRouteDistance)
            {
                var valor = route[i];
                route[i] = route[j];
                route[j] = valor;

                if (isValidRoute(route))
                {
                    _customerSolution.totalRouteDistance = nuevo_costo;
                    //_clusterSolution.clusterRouteForVehicule[vehicle] = route;
                    return true;
                }
            }
            return false;
        }*/
           
    }
}
