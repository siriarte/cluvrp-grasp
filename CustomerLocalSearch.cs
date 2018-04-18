using System.Collections.Generic;
using System.Linq;

namespace cluvrp_grasp
{

    class CustomerLocalSearch
    {
        public int _maxIterationsWithoutImprovementTwoOpt { get; set; }
        public int _maxIterationsWithoutImprovementRelocate { get; set; }
        public int _maxIterationsWithoutImprovementExchange { get; set; }
        public CustomerSolution _customerSolution { get; set; }
        public double[][] _customersMatrixDistance { get; set; }

        public CustomerLocalSearch(CustomerSolution clusterSolution, double[][] clusterMatrixDistance,
            int maxIterationsWithoutImprovement = 100,
            int maxIterationsWithoutImprovementRelocate = 100,
            int maxIterationsWithoutImprovementExchange = 100)
        {
            _maxIterationsWithoutImprovementTwoOpt = maxIterationsWithoutImprovement;
            _customerSolution = clusterSolution;
            _customersMatrixDistance = clusterMatrixDistance;
            _maxIterationsWithoutImprovementRelocate = maxIterationsWithoutImprovementRelocate;
            _maxIterationsWithoutImprovementExchange = maxIterationsWithoutImprovementExchange;
        }

        public void twoOpt()
        {
            List<int>[][] customersCircuit = _customerSolution._customersCircuit;
            int numberOfVehicles = customersCircuit.Length;
            double[] bestDistance = new double[numberOfVehicles];

            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {

                List<int>[] clusterRoute = customersCircuit[vehicle];
                int iteration = 0;             
                bestDistance[vehicle] = Functions.calculateTotalTravelDistance(customersCircuit, this._customersMatrixDistance, vehicle);

                while (iteration < _maxIterationsWithoutImprovementTwoOpt)
                {
                    for (int clusterIt = 0; clusterIt < clusterRoute.Length; clusterIt++)
                    {
                        List<int> route = clusterRoute[clusterIt];
                        int max_w = route.Count();

                        for (int i = 1; i < max_w - 1; i++)
                        {
                            for (int k = i + 1; k < max_w; k++)
                            {
                                List<int> oldRoute = customersCircuit[vehicle][clusterIt];
                                List<int> newRoute = twoOptSwap(customersCircuit[vehicle][clusterIt], i, k);
                                customersCircuit[vehicle][clusterIt] = newRoute;
                                double newDistance = Functions.calculateTotalTravelDistance(customersCircuit, this._customersMatrixDistance, vehicle);
                                if (newDistance < bestDistance[vehicle])
                                {
                                    iteration = 0;
                                    customersCircuit[vehicle][clusterIt] = newRoute;
                                    bestDistance[vehicle] = newDistance;
                                }else
                                {
                                    customersCircuit[vehicle][clusterIt] = oldRoute;
                                }
                            }
                        }
                        iteration++;
                    }
                }
            }

            this._customerSolution._customersCircuit = customersCircuit;
            this._customerSolution.vehiculeRouteDistance = bestDistance;
        }
        
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
        
        public void relocate()
        {
            int numberOfVehicles = _customerSolution._customersCircuit.Length;

            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                int numberOfClusters = _customerSolution._customersCircuit[vehicle].Length;

                for (int clusterIt = 0; clusterIt < numberOfClusters; clusterIt++)
                {
                    int iteration = 0;
                    List<int> route = _customerSolution._customersCircuit[vehicle][clusterIt];
                    while (iteration < _maxIterationsWithoutImprovementRelocate)
                    {
                        for (int i = 0; i < route.Count; i++)
                        {
                            for (int j = 0; j < route.Count; j++)
                            {
                                if ((i == j) || ((i == 0 && j == route.Count - 1) ||
                                    (i == route.Count - 1 && j == 0)))
                                    continue;

                                if (relocate(vehicle, clusterIt, i, j))
                                {
                                    iteration = 0;
                                }
                            }
                        }
                        iteration++;
                    }
                }
            }
        }

        public bool relocate(int vehicle, int cluster, int i, int j)
        {
            List<int> route = _customerSolution._customersCircuit[vehicle][cluster];

            // Para no irnos del camino
            var celda_anterior_i = i - 1 == -1 ? route.Count - 1 : i - 1;
            var celda_anterior_j = j - 1 == -1 ? route.Count - 1 : j - 1;

            var celda_siguiente_i = i + 1 == route.Count ? 0 : i + 1;
            var celda_siguiente_j = j + 1 == route.Count ? 0 : j + 1;

            if (i < j)
                celda_anterior_j++;
            else
                celda_siguiente_j--;

            // Calculamos el nuevo costo, a ver si es mejor cambiar o no de posición 
            List<int> oldRoute = new List<int>(route);
            var valor = route[i];
            route.RemoveAt(i);
            route.Insert(j, valor);
            _customerSolution._customersCircuit[vehicle][cluster] = route;

            var nuevo_costo = Functions.calculateTotalTravelDistance(_customerSolution._customersCircuit, _customersMatrixDistance, vehicle);
            if (nuevo_costo < _customerSolution.vehiculeRouteDistance[vehicle])
            {
                _customerSolution.vehiculeRouteDistance[vehicle] = nuevo_costo;
                return true;
            }
             
             _customerSolution._customersCircuit[vehicle][cluster] = oldRoute;
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

        public bool isValidRoute(List<int> route)
        {
            if (route.Count > 0)
            {
                return (route[0] == 0 && route[route.Count - 1] == 0);
            }
            return false;
        }
    }
}
