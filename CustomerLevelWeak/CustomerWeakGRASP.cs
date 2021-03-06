﻿using System;
using System.Collections.Generic;
using System.Linq;

namespace cluvrp_grasp
{

    class CustomerWeakGRASP :  CluVRP.CustomerLevel
    {
        // Public variables
        public CluVRPInstance instance;
        public CluVRPSolution solution;
        public Parameters parameters;
        public List<LocalSearch> localSearchsOrder;

        /*
         * 
         * Constructor 
         *
         */
        public CustomerWeakGRASP(CluVRPInstance instance, CluVRPSolution solution, Parameters parameters)
        {
            // Set variables
            this.instance = instance;
            this.solution = solution;
            this.parameters = parameters;

            // For local search execute order
            if (parameters.Customer_LS_Order.Length == 0)
            {
                this.localSearchsOrder = new List<LocalSearch> { LocalSearch.TwoOpt,
                LocalSearch.Relocate, LocalSearch.Exchange, LocalSearch.SwapCustomers};
            }
            else
            {
                this.localSearchsOrder = new List<LocalSearch>();
                for (int i = 0; i < parameters.Customer_LS_Order.Length; i++)
                {
                    localSearchsOrder.Add((LocalSearch)parameters.Customer_LS_Order[i]);
                }
            }

            // End of constructor
        }

        /*
        * 
        * Grasp():
        * M <- calculateCustomerDistance
        * BestSolution = 0
        * While(StopCondition)
        *      Solution <- ConstructGreedySolution()
        *      NewSolution <- LocalSearch(solution)
        *      if NewSolution isBetterThan BestSolution
        *          BestSolution = NewSolution
        * return BestSolution
        *
        */
        override public void Grasp()
        {
            // Set iterator
            int iterator = 0;
            int totalIterations = parameters.Customer_GRASPIterations;
            double alpha = parameters.Customer_Alpha;

            // Create a customer path for each vehicle 
            List<int>[] customersOnVehicle = buildCustomersOnVehicle();

            // Main cycle
            while (iterator < totalIterations)
            {
                // For random alpha
                if (parameters.Customer_Alpha == -1)
                {
                    Random rnd = new Random();
                    alpha = rnd.Next(0, 11) / 10.0;
                }

                // Calculate new initial solution
                CluVRPSolution newSolution = constructGreedyRandomizedSolution(customersOnVehicle, alpha);

                // Local search 
                var totalLSWatch = System.Diagnostics.Stopwatch.StartNew();
                double routeDistance = newSolution.totalCustomerRouteDistance;
                for (int i = 0; i < 1; i++)
                {
                    this.localSearch(newSolution);

                    // For control of best iteration number
                    if (newSolution.totalCustomerRouteDistance < routeDistance)
                    {
                        newSolution.cluster_LSCycle_iterations = i;
                        routeDistance = newSolution.totalCustomerRouteDistance;
                    }
                }
                newSolution.customer_LSCycleTime += totalLSWatch.ElapsedMilliseconds;

                // Update Best solution
                if (newSolution.totalCustomerRouteDistance < solution.totalCustomerRouteDistance)
                {
                    solution.setWeakCostumerSolution(newSolution.customersWeakRoute, newSolution.vehiculeRouteDistance);
                    solution.bestCustomerLSOrder = localSearchsOrder;

                    solution.customerLevelIterations = iterator;
                    solution.customer_twoOpt_iterations = newSolution.customer_twoOpt_iterations;
                    solution.customer_relocate_iterations = newSolution.customer_relocate_iterations;
                    solution.customer_exchange_iterations = newSolution.customer_exchange_iterations;
                    solution.customer_swapCustomers_iterations = newSolution.customer_swapCustomers_iterations;

                    solution.customer_LSCycleTime = newSolution.customer_LSCycleTime;
                    solution.customer_twoOpt_time = newSolution.customer_twoOpt_time;
                    solution.customer_relocate_time = newSolution.customer_relocate_time;
                    solution.customer_exchange_time = newSolution.customer_exchange_time;
                    solution.customer_swapCustomers_time = newSolution.customer_swapCustomers_time;
                }

                // Increace iterator
                iterator++;
            }

            //End
            return;
        }

        /*
         *
         * Group all customer for each cluster for each vehicle
         *
         *
         */
        private List<int>[] buildCustomersOnVehicle()
        {
            // Init variables
            int vehicleNumber = solution.clusterRouteForVehicule.Length;
            List<int>[] customersOnVehicle = new List<int>[vehicleNumber];  

            // For each vehicle
            for(int vehicle = 0; vehicle < vehicleNumber; vehicle++)
            {
                // New list of customers for all clusters on vehicle
                customersOnVehicle[vehicle] = new List<int>();
                
                // For each clusters
                for (int clusterIt = 1; clusterIt + 1 < solution.clusterRouteForVehicule[vehicle].Count; clusterIt++)
                {
                    // Add customer of cluster to the list
                    int cluster = solution.clusterRouteForVehicule[vehicle][clusterIt];
                    customersOnVehicle[vehicle].AddRange(instance.clusters[cluster].ToList<int>());
                }
            }

            // End
            return customersOnVehicle;
        }

        /*
         * 
         * Applies a list of local searchs  
         *
         */
        private void localSearch(CluVRPSolution newSolution)
        {
            // Create a local search handler for cluster-level problem
            CustomerWeakLocalSearch customerLocalSearch = new CustomerWeakLocalSearch(newSolution,
                instance,
                parameters.Customer_LS_TwoOpt_Iterations,
                parameters.Customer_LS_Relocate_Iterations,
                parameters.Customer_LS_Exchange_Iterations,
                parameters.Customer_LS_SwapCustomers
                );


            // If random order for local searchs is activated
            if (parameters.Customer_LS_Order.Length == 0)
            {
                Functions.Shuffle(new Random(), this.localSearchsOrder);
            }

            // Execute local search in the correct order
            for (int i = 0; i < localSearchsOrder.Count; i++)
            {
                // Perform TwoOpt
                if (localSearchsOrder[i] == LocalSearch.TwoOpt && parameters.Customer_LS_TwoOpt_Iterations != 0)
                {
                    var totalWatch = System.Diagnostics.Stopwatch.StartNew();
                    customerLocalSearch.twoOpt();
                    customerLocalSearch.solution.customer_twoOpt_time += totalWatch.ElapsedMilliseconds;
                }

                // Perform Relocate
                if (localSearchsOrder[i] == LocalSearch.Relocate && parameters.Customer_LS_Relocate_Iterations != 0)
                {
                    var totalWatch = System.Diagnostics.Stopwatch.StartNew();
                    customerLocalSearch.relocate();
                    customerLocalSearch.solution.customer_relocate_time += totalWatch.ElapsedMilliseconds;
                }

                // Perform Exchange
                if (localSearchsOrder[i] == LocalSearch.Exchange && parameters.Customer_LS_Exchange_Iterations != 0)
                {
                    var totalWatch = System.Diagnostics.Stopwatch.StartNew();
                    customerLocalSearch.exchange();
                    customerLocalSearch.solution.customer_exchange_time += totalWatch.ElapsedMilliseconds;
                }

                // Perform Customer Swap
                if (localSearchsOrder[i] == LocalSearch.SwapCustomers && parameters.Customer_LS_SwapCustomers != 0)
                {
                    var totalWatch = System.Diagnostics.Stopwatch.StartNew();
                    customerLocalSearch.swapCustomers();
                    customerLocalSearch.solution.customer_swapCustomers_time += totalWatch.ElapsedMilliseconds;
                }

            }
            // Set the solution
            newSolution = customerLocalSearch.solution;
        }

        /*
         * 
         *  Create a complete Greedy Randomized Solution (with cluster and customers)
         * 
         */
        private CluVRPSolution constructGreedyRandomizedSolution(List<int>[] customersOnVehicle, double alpha)
        {
            // Init variables
            int vehiclesNumber = customersOnVehicle.Length;
            List<int>[] customersCircuit = new List<int>[vehiclesNumber];
            double[] vehiculeTotalDistance = new double[vehiclesNumber];            

            // For each vehicule cluster-route
            for (int vehicle = 0; vehicle < customersCircuit.Length; vehicle++)
            {
                // Init customer circuit for i-vehicle 
                List<int> customersToVisit = customersOnVehicle[vehicle].ToList<int>();
                customersCircuit[vehicle] = new List<int>();

                // Add depot as first customer
                customersCircuit[vehicle].Add(1);

                // While exists customers to visit
                while (customersToVisit.Count > 0)
                {
                    // Last customer
                    int lastCustomer = customersCircuit[vehicle][customersCircuit[vehicle].Count - 1];

                    // Create RCL for customer 
                    List<int> customerRCL = buildCustomerRCL(customersToVisit, lastCustomer, alpha);

                    // Select customer for RCL
                    int customerSelected = Functions.selectRandomElement(customerRCL);

                    // Add customer to the path
                    customersCircuit[vehicle].Add(customerSelected);

                    // Quit visited customer
                    customersToVisit.Remove(customerSelected);
                }

                // Add depot as final customer
                customersCircuit[vehicle].Add(1);

                // Calculte total inter-cluster distance
                vehiculeTotalDistance[vehicle] = Functions.calculateCustomerTravelDistance(customersCircuit[vehicle], instance.customersDistanceMatrix);
            }

            // Set solution
            CluVRPSolution newSolution = new CluVRPSolution(instance);
            newSolution.setWeakCostumerSolution(customersCircuit, vehiculeTotalDistance);

            // Return solution
            return newSolution;
        }
                
        /*
         * 
         * Create RCL list with min distance betweeen customers as criteria 
         * 
         */
        private List<int> buildCustomerRCL(List<int> customersToVisit,  int lastCustomer, double alpha)
        {
            // Set variables
            List<int> RCL = new List<int>();

            // Calculate max and min distance for RCL condition
            double minDistance = Functions.minCustomerDistance(customersToVisit, lastCustomer, instance.customersDistanceMatrix);
            double maxDistance = Functions.maxCustomerDistance(customersToVisit, lastCustomer, instance.customersDistanceMatrix);

            // Set RCL condition criteria
            double RCLCondition = minDistance + alpha * (maxDistance - minDistance);

            // For each vehicle
            for (int j = 0; j < customersToVisit.Count; j++)
            {
                // Calculate customers distance
                double distanceBetweenCustomers = instance.customersDistanceMatrix[lastCustomer][customersToVisit[j]];

                // Add the vehicle to RCL if is possible
                if (distanceBetweenCustomers <= RCLCondition)
                {
                    RCL.Add(customersToVisit[j]);
                }
            }

            // return rcl
            return RCL;
        }
        
    }
}
