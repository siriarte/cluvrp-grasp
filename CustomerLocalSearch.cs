using System.Collections.Generic;
using System.Linq;

namespace cluvrp_grasp
{

    class CustomerLocalSearch
    {
        public int _maxIterationsWithoutImprovementTwoOpt { get; set; }
        public int _maxIterationsWithoutImprovementRelocate { get; set; }
        public int _maxIterationsWithoutImprovementExchange { get; set; }
        public CustomerSolution _clusterSolution { get; set; }
        public double[][] _clusterMatrixDistance { get; set; }

        public CustomerLocalSearch(CustomerSolution clusterSolution, double[][] clusterMatrixDistance,
            int maxIterationsWithoutImprovement = 100,
            int maxIterationsWithoutImprovementRelocate = 100,
            int maxIterationsWithoutImprovementExchange = 100)
        {
            _maxIterationsWithoutImprovementTwoOpt = maxIterationsWithoutImprovement;
            _clusterSolution = clusterSolution;
            _clusterMatrixDistance = clusterMatrixDistance;
            _maxIterationsWithoutImprovementRelocate = maxIterationsWithoutImprovementRelocate;
            _maxIterationsWithoutImprovementExchange = maxIterationsWithoutImprovementExchange;
        }

        public void twoOpt()
        {
            List<int>[] customersPaths = _clusterSolution.clusterRouteForVehicule;
            int numberOfClusters = customersPaths.Length;
            double[] bestDistance = new double[numberOfClusters];

            for (int vehicle = 0; vehicle < numberOfClusters; vehicle++)
            {

                List<int> route = customersPaths[vehicle];
                int iteration = 0;
                int max_w = route.Count();

                bestDistance[vehicle] = clusterTravelDistance(route, this._clusterMatrixDistance);

                while (iteration < _maxIterationsWithoutImprovementTwoOpt)
                {
                    for (int i = 1; i < max_w - 1; i++)
                    {
                        for (int k = i + 1; k < max_w; k++)
                        {
                            List<int> newRoute = twoOptSwap(route, i, k);
                            double newDistance = clusterTravelDistance(newRoute, this._clusterMatrixDistance);
                            if (newDistance < bestDistance[vehicle] && isValidRoute(newRoute))
                            {
                                iteration = 0;
                                customersPaths[vehicle] = newRoute;
                                bestDistance[vehicle] = newDistance;
                            }
                        }
                    }
                    iteration++;
                }
            }

            this._clusterSolution = new CustomerSolution(customersPaths, bestDistance.Sum());
        }
        /*
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
            int numberOfVehicles = _clusterSolution.clusterRouteForVehicule.Length;

            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                List<int> route = _clusterSolution.clusterRouteForVehicule[vehicle];
                int iteration = 0;
                while (iteration < _maxIterationsWithoutImprovementRelocate)
                {

                    for (int i = 0; i < route.Count; i++)
                    {
                        for (int j = 0; j < route.Count; j++)
                        {
                            if ((i == j) || ((i == 0 && j == route.Count - 1) ||
                                (i == route.Count - 1 && j == 0)))
                                continue;

                            if (relocate(route, vehicle, i, j))
                            {
                                iteration = 0;
                            }
                        }
                    }
                    iteration++;
                }
            }
        }

        public bool relocate(List<int> route, int vehicle, int i, int j)
        {
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
            var _a = _clusterMatrixDistance[route[celda_anterior_i]][route[i]];
            var _b = _clusterMatrixDistance[route[i]][route[celda_siguiente_i]];
            var _C = _clusterMatrixDistance[route[celda_anterior_i]][route[celda_siguiente_i]];

            var _A = _clusterMatrixDistance[route[celda_anterior_j]][route[i]];
            var _B = _clusterMatrixDistance[route[i]][route[celda_siguiente_j]];
            var _c = _clusterMatrixDistance[route[celda_anterior_j]][route[celda_siguiente_j]];

            var nuevo_costo = _clusterSolution.totalRouteDistance - _a - _b + _C + _A + _B - _c;
            if (nuevo_costo < _clusterSolution.totalRouteDistance)
            {
                var valor = route[i];
                route.RemoveAt(i);
                route.Insert(j, valor);

                if (isValidRoute(route))
                {
                    _clusterSolution.totalRouteDistance = nuevo_costo;
                    _clusterSolution.clusterRouteForVehicule[vehicle] = route;
                    return true;
                }
                else
                {
                    route.RemoveAt(j);
                    route.Insert(i, valor);
                }
            }
            return false;
        }

        public static double clusterTravelDistance(List<int> travel, double[][] clustersDistanceMatrix)
        {
            // Set variables
            double totalDistance = 0;

            // Iterate each cluster on vehicle route
            for (int clusterIt = 0; clusterIt + 1 < travel.Count; clusterIt++)
            {
                int fromCluster = travel[clusterIt];
                int ToCluster = travel[clusterIt + 1];
                totalDistance += clustersDistanceMatrix[fromCluster][ToCluster];
            }

            // Return total distance
            return totalDistance;
        }

        public void exchange()
        {
            int numberOfVehicles = _clusterSolution.clusterRouteForVehicule.Length;

            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                List<int> route = _clusterSolution.clusterRouteForVehicule[vehicle];
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
            var _distancia_i_izquierda = _clusterMatrixDistance[route[celda_anterior_i]][route[i]];
            var _distancia_i_derecha = _clusterMatrixDistance[route[i]][route[celda_siguiente_i]];

            var _distancia_j_izquierda = _clusterMatrixDistance[route[celda_anterior_j]][route[j]];
            var _distancia_j_derecha = _clusterMatrixDistance[route[j]][route[celda_siguiente_j]];

            double nuevo_costo = 0;
            // Costos nuevos
            double _distancia_nueva_i_izquierda, _distancia_nueva_i_derecha, _distancia_nueva_j_izquierda, _distancia_nueva_j_derecha;

            if (i == celda_siguiente_j)
            {
                _distancia_nueva_i_izquierda = _clusterMatrixDistance[route[celda_anterior_j]][route[i]];
                _distancia_nueva_i_derecha = _clusterMatrixDistance[route[i]][route[j]];

                _distancia_nueva_j_derecha = _clusterMatrixDistance[route[j]][route[celda_siguiente_i]];
                _distancia_nueva_j_izquierda = _clusterMatrixDistance[route[j]][route[i]];

                //_distancia_nueva_i_derecha = _distancia_nueva_j_izquierda 
                //    = _clusterMatrixDistance[solucion.camino[i], solucion.camino[j]).distancia;
            }
            else if (j == celda_siguiente_i)
            {
                _distancia_nueva_i_derecha = _clusterMatrixDistance[route[i]][route[celda_siguiente_j]];
                _distancia_nueva_i_izquierda = _clusterMatrixDistance[route[j]][route[i]];

                _distancia_nueva_j_izquierda = _clusterMatrixDistance[route[celda_anterior_i]][route[j]];
                _distancia_nueva_j_derecha = _clusterMatrixDistance[route[i]][route[j]];

                //_distancia_nueva_i_izquierda = _distancia_nueva_j_derecha = 
                //    _clusterMatrixDistance[solucion.camino[j], solucion.camino[i]).distancia;
            }
            else
            {
                _distancia_nueva_i_izquierda = _clusterMatrixDistance[route[celda_anterior_j]][route[i]];
                _distancia_nueva_i_derecha = _clusterMatrixDistance[route[i]][route[celda_siguiente_j]];

                _distancia_nueva_j_izquierda = _clusterMatrixDistance[route[celda_anterior_i]][route[j]];
                _distancia_nueva_j_derecha = _clusterMatrixDistance[route[j]][route[celda_siguiente_i]];
            }

            nuevo_costo = this._clusterSolution.totalRouteDistance - _distancia_i_izquierda - _distancia_i_derecha - _distancia_j_izquierda - _distancia_j_derecha +
            _distancia_nueva_i_izquierda + _distancia_nueva_i_derecha + _distancia_nueva_j_izquierda + _distancia_nueva_j_derecha;

            if (nuevo_costo < this._clusterSolution.totalRouteDistance)
            {
                var valor = route[i];
                route[i] = route[j];
                route[j] = valor;

                if (isValidRoute(route))
                {
                    _clusterSolution.totalRouteDistance = nuevo_costo;
                    //_clusterSolution.clusterRouteForVehicule[vehicle] = route;
                    return true;
                }
            }
            return false;
        }

        public bool isValidRoute(List<int> route)
        {
            if (route.Count > 0)
            {
                return (route[0] == 0 && route[route.Count - 1] == 0);
            }
            return false;
        }*/
    }
}
