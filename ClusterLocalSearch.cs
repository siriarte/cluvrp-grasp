using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace cluvrp_grasp
{
          
    class ClusterLocalSearch
    {
        public int maxIterationsWithoutImprovementTwoOpt { get; set; }
        public int maxIterationsWithoutImprovementRelocate { get; set; }
        public int maxIterationsWithoutImprovementExchange { get; set; }
        public CluVRPSolution solution { get; set; }
        public CluVRPInstance instance;

        public ClusterLocalSearch(CluVRPSolution solution,
            CluVRPInstance instance,
            int maxIterationsWithoutImprovement = 100,
            int maxIterationsWithoutImprovementRelocate = 100,
            int maxIterationsWithoutImprovementExchange = 100
            )
        {
            this.solution = solution;
            this.instance = instance;
            this.maxIterationsWithoutImprovementTwoOpt = maxIterationsWithoutImprovement;
            this.maxIterationsWithoutImprovementRelocate = maxIterationsWithoutImprovementRelocate;
            this.maxIterationsWithoutImprovementExchange = maxIterationsWithoutImprovementExchange;
        }

        public void twoOpt()
        {
            List<int>[] routeForVehicule = solution.clusterRouteForVehicule;
            int numberOfVehicles = routeForVehicule.Length;
            double[] bestDistance = new double[numberOfVehicles];

            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++) {

                List<int> route = routeForVehicule[vehicle];
                int iteration = 0;
                int max_w = route.Count();

                bestDistance[vehicle] = ClusterGRASP.calculateClusterTravelDistance(route, instance.clustersDistanceMatrix);

                while (iteration < maxIterationsWithoutImprovementTwoOpt)
                {
                    for(int i = 1; i < max_w - 1; i++)
                    {
                        for(int k = i + 1; k < max_w; k++)
                        {
                            List<int> newRoute = twoOptSwap(route, i, k);
                            double newDistance = ClusterGRASP.calculateClusterTravelDistance(newRoute, instance.clustersDistanceMatrix);
                            if (newDistance + 0.5 < bestDistance[vehicle] && isValidRoute(newRoute))
                            {
                                iteration = 0;
                                routeForVehicule[vehicle] = newRoute;
                                bestDistance[vehicle] = newDistance;
                            }
                        }
                    }
                    iteration++;
                }
            }

            this.solution.clusterRouteForVehicule = routeForVehicule;
            this.solution.totalClusterRouteDistance = bestDistance.Sum();
        }

        public List<int> twoOptSwap(List<int> route, int i, int k)
        {
            List<int> newRoute = route.GetRange(0, i);
            int reverseSize = k - i + 1;
            List<int> reverseRoute = route.GetRange(i, reverseSize);
            reverseRoute.Reverse();
            int restSize = route.Count - (k + 1);
            List<int> endRoute = route.GetRange(k+1, restSize);
            newRoute.AddRange(reverseRoute);
            newRoute.AddRange(endRoute);
            return newRoute;
        }

        public void relocate()
        {
            int numberOfVehicles = solution.clusterRouteForVehicule.Length;

            for(int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                List<int> route = solution.clusterRouteForVehicule[vehicle];
                int iteration = 0;
                while (iteration < maxIterationsWithoutImprovementRelocate)
                {

                    for (int i = 0; i < route.Count; i++)
                    {
                        for (int j = 0; j < route.Count; j++)
                        {
                            if ((i == j) || ((i == 0 && j == route.Count - 1) ||
                                (i == route.Count - 1 && j == 0)))
                                continue;

                            if(relocate(route, vehicle, i, j))
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
            var _a = instance.clustersDistanceMatrix[route[celda_anterior_i]][route[i]];
            var _b = instance.clustersDistanceMatrix[route[i]][route[celda_siguiente_i]];
            var _C = instance.clustersDistanceMatrix[route[celda_anterior_i]][route[celda_siguiente_i]];

            var _A = instance.clustersDistanceMatrix[route[celda_anterior_j]][route[i]];
            var _B = instance.clustersDistanceMatrix[route[i]][route[celda_siguiente_j]];
            var _c = instance.clustersDistanceMatrix[route[celda_anterior_j]][route[celda_siguiente_j]];

            var nuevo_costo = solution.totalClusterRouteDistance - _a - _b + _C + _A + _B - _c;
            if (nuevo_costo < solution.totalClusterRouteDistance)
            {
                var valor = route[i];
                route.RemoveAt(i);
                route.Insert(j, valor);

                if (isValidRoute(route))
                {
                    solution.totalClusterRouteDistance = nuevo_costo;
                    solution.clusterRouteForVehicule[vehicle] = route;
                    return true;
                }else
                {
                    route.RemoveAt(j);
                    route.Insert(i, valor);
                }
            }
            return false;
        }
                  
        public void exchange()
        {
            int numberOfVehicles = solution.clusterRouteForVehicule.Length;

            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                List<int> route = solution.clusterRouteForVehicule[vehicle];
                int iteration = 0;
                while (iteration < maxIterationsWithoutImprovementExchange)
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
            var _distancia_i_izquierda = instance.clustersDistanceMatrix[route[celda_anterior_i]][route[i]];
            var _distancia_i_derecha = instance.clustersDistanceMatrix[route[i]][route[celda_siguiente_i]];

            var _distancia_j_izquierda = instance.clustersDistanceMatrix[route[celda_anterior_j]][route[j]];
            var _distancia_j_derecha = instance.clustersDistanceMatrix[route[j]][route[celda_siguiente_j]];

            double nuevo_costo = 0;
            // Costos nuevos
            double _distancia_nueva_i_izquierda, _distancia_nueva_i_derecha, _distancia_nueva_j_izquierda, _distancia_nueva_j_derecha;

            if (i == celda_siguiente_j)
            {
                _distancia_nueva_i_izquierda = instance.clustersDistanceMatrix[route[celda_anterior_j]][route[i]];
                _distancia_nueva_i_derecha = instance.clustersDistanceMatrix[route[i]][route[j]];

                _distancia_nueva_j_derecha = instance.clustersDistanceMatrix[route[j]][route[celda_siguiente_i]];
                _distancia_nueva_j_izquierda = instance.clustersDistanceMatrix[route[j]][route[i]];

                //_distancia_nueva_i_derecha = _distancia_nueva_j_izquierda 
                //    = _clusterMatrixDistance[solucion.camino[i], solucion.camino[j]).distancia;
            }
            else if (j == celda_siguiente_i)
            {
                _distancia_nueva_i_derecha = instance.clustersDistanceMatrix[route[i]][route[celda_siguiente_j]];
                _distancia_nueva_i_izquierda = instance.clustersDistanceMatrix[route[j]][route[i]];

                _distancia_nueva_j_izquierda = instance.clustersDistanceMatrix[route[celda_anterior_i]][route[j]];
                _distancia_nueva_j_derecha = instance.clustersDistanceMatrix[route[i]][route[j]];

                //_distancia_nueva_i_izquierda = _distancia_nueva_j_derecha = 
                //    _clusterMatrixDistance[solucion.camino[j], solucion.camino[i]).distancia;
            }
            else
            {
                _distancia_nueva_i_izquierda = instance.clustersDistanceMatrix[route[celda_anterior_j]][route[i]];
                _distancia_nueva_i_derecha = instance.clustersDistanceMatrix[route[i]][route[celda_siguiente_j]];

                _distancia_nueva_j_izquierda = instance.clustersDistanceMatrix[route[celda_anterior_i]][route[j]];
                _distancia_nueva_j_derecha = instance.clustersDistanceMatrix[route[j]][route[celda_siguiente_i]];
            }

            nuevo_costo = this.solution.totalClusterRouteDistance - _distancia_i_izquierda - _distancia_i_derecha - _distancia_j_izquierda - _distancia_j_derecha +
            _distancia_nueva_i_izquierda + _distancia_nueva_i_derecha + _distancia_nueva_j_izquierda + _distancia_nueva_j_derecha;

            if (nuevo_costo + 0.5 < this.solution.totalClusterRouteDistance)
            {
                var valor = route[i];
                route[i] = route[j];
                route[j] = valor;

                if (isValidRoute(route))
                {
                    solution.totalClusterRouteDistance = nuevo_costo;
                    solution.clusterRouteForVehicule[vehicle] = route;
                    return true;
                }
            }
            return false;
        }

        public bool isValidRoute(List<int> route)
        {
            if (route.Count > 0) {
                return (route[0] == 0 && route[route.Count - 1] == 0);
            }
            return false;
        }

        public void interVehicleRandomSwap()
        {
            int iterator = 0;
            while (iterator < maxIterationsWithoutImprovementExchange)
            {
                Random rnd = new Random();
                int numberOfVehicles = solution.clusterRouteForVehicule.Length;
                int vehicle1 = rnd.Next(0, numberOfVehicles);
                int vehicle2 = rnd.Next(0, numberOfVehicles);
                int clusterV1 = Functions.selectRandomElement(solution.clusterRouteForVehicule[vehicle1]);
                int clusterV2 = Functions.selectRandomElement(solution.clusterRouteForVehicule[vehicle2]);
                if (clusterV1 != 0 && clusterV2 != 0 && vehicle1 != vehicle2)
                {
                    int idxClusterV1 = solution.clusterRouteForVehicule[vehicle1].IndexOf(clusterV1);
                    int idxClusterV2 = solution.clusterRouteForVehicule[vehicle2].IndexOf(clusterV2);
                    solution.clusterRouteForVehicule[vehicle1][idxClusterV1] = clusterV2;
                    solution.clusterRouteForVehicule[vehicle2][idxClusterV2] = clusterV1;
                    double newDistance = ClusterGRASP.calculateClusterTravelDistance(solution.clusterRouteForVehicule, instance.clustersDistanceMatrix);
                    if (newDistance < solution.totalClusterRouteDistance)
                    {
                        solution.totalClusterRouteDistance = newDistance;
                        iterator = 0;
                    }
                    else
                    {
                        solution.clusterRouteForVehicule[vehicle1][idxClusterV1] = clusterV1;
                        solution.clusterRouteForVehicule[vehicle2][idxClusterV2] = clusterV2;
                        iterator++;
                    }
                }
            }
            return;
        }

        public void interVehicleRandomInsert(int[] clusterDemand)
        {
            int iterator = 0;
            while (iterator < maxIterationsWithoutImprovementExchange)
            {
                Random rnd = new Random();
                int numberOfVehicles = solution.clusterRouteForVehicule.Length;
                int vehicle1 = rnd.Next(0, numberOfVehicles);
                int vehicle2 = rnd.Next(0, numberOfVehicles);
                int clusterV1 = Functions.selectRandomElement(solution.clusterRouteForVehicule[vehicle1]);
                bool improve = false;
                if (clusterV1 !=0 && vehicle1 != vehicle2 && solution.vehicleRemSpace[vehicle2] - clusterDemand[clusterV1] >= 0)
                {
                    int idxClusterV1 = solution.clusterRouteForVehicule[vehicle1].IndexOf(clusterV1);
                    solution.clusterRouteForVehicule[vehicle1].Remove(clusterV1);
                    for (int i = 0; i < solution.clusterRouteForVehicule[vehicle2].Count; i++)
                    {
                        solution.clusterRouteForVehicule[vehicle2].Insert(i + 1, clusterV1);
                        double newDistance = ClusterGRASP.calculateClusterTravelDistance(solution.clusterRouteForVehicule, instance.clustersDistanceMatrix);
                        if (newDistance < solution.totalClusterRouteDistance)
                        {
                            solution.totalClusterRouteDistance = newDistance;
                            solution.vehicleRemSpace[vehicle1] += clusterDemand[clusterV1];
                            solution.vehicleRemSpace[vehicle2] -= clusterDemand[clusterV1];
                            iterator = 0;
                            improve = true;
                            break;
                        }
                        else
                        {
                            solution.clusterRouteForVehicule[vehicle2].Remove(clusterV1);
                        }
                    }
                    if (!improve)
                    {
                        solution.clusterRouteForVehicule[vehicle1].Insert(idxClusterV1, clusterV1);
                        improve = false;
                    }                    
                }
                iterator++;
            }
            return;
        }

    }
}
