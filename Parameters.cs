using Newtonsoft.Json;
using System.Collections.Generic;
using System.IO;

namespace cluvrp_grasp
{
    class Parameters
    {
        // Main Settings
        public CluVRPVersion CluVRP_Version { get; set; }
        public int CluVRP_GRASPIterations { get; set; }
        public int CluVRP_LS_Main_Iterations { get; set; }

        // CluVRP level
        public int CluVRP_LS_SwapClusters { get; set; }
        public int CluVRP_LS_SwapVehicle { get; set; }

        // Cluster level
        public int Cluster_GRASPIterations { get; set; }
        public double Cluster_AlphaCapacity { get; set; }
        public double Cluster_AlphaDistance { get; set; }
        public FitAlgorithm Cluster_FitAlgoritm { get; set; }
        public int[] Cluster_LS_Order { get; set; }
        public int Cluster_LS_SwapClusters { get; set; }
        public int Cluster_LS_SwapVehicle { get; set; }
        public int Cluster_LS_InsertVehicle { get; set; }
        public int Cluster_LS_TwoOpt_Iterations { get; set; }
        public int Cluster_LS_Relocate_Iterations { get; set; }
        public int Cluster_LS_Exchange_Iterations { get; set; }

        // Customer level
        public int Customer_GRASPIterations { get; set; }
        public double Customer_Alpha { get; set; }
        public int[] Customer_LS_Order { get; set; }
        public int Customer_LS_SwapCustomers { get; set; }
        public int Customer_LS_TwoOpt_Iterations { get; set; }
        public int Customer_LS_Relocate_Iterations { get; set; }
        public int Customer_LS_Exchange_Iterations { get; set; }

        // Constructor
        public Parameters(
            CluVRPVersion CluVRP_Version,
            int CluVRP_GRASPIterations,
            int CluVRP_LS_Main_Iterations,
            int CluVRP_LS_SwapClusters,
            int CluVRP_LS_SwapVehicle,
            int Cluster_GRASPIterations,
            double Cluster_AlphaCapacity,
            double Cluster_AlphaDistance,
            FitAlgorithm Cluster_FitAlgoritm,
            int[] Cluster_LS_Order,
            int Cluster_LS_SwapVehicle,
            int Cluster_LS_InsertVehicle,
            int Cluster_LS_SwapClusters,
            //int Cluster_LS_RndSwapVehicle,
            //int Cluster_LS_RndInsertVehicle,
            int Cluster_LS_TwoOpt_Iterations,
            int Cluster_LS_Relocate_Iterations,
            int Cluster_LS_Exchange_Iterations,
            int Customer_GRASPIterations,
            double Customer_Alpha,
            int[] Customer_LS_Order,
            int Customer_LS_SwapCustomers,
            int Customer_LS_TwoOpt_Iterations,
            int Customer_LS_Relocate_Iterations,
            int Customer_LS_Exchange_Iterations)
        {
            // CluVRP
            this.CluVRP_Version = CluVRP_Version;
            this.CluVRP_GRASPIterations = CluVRP_GRASPIterations;
            this.CluVRP_LS_Main_Iterations = CluVRP_LS_Main_Iterations;
            this.CluVRP_LS_SwapClusters = CluVRP_LS_SwapClusters;
            this.CluVRP_LS_SwapVehicle = CluVRP_LS_SwapVehicle;

            // Cluster Level
            this.Cluster_GRASPIterations = Cluster_GRASPIterations;
            this.Cluster_AlphaCapacity = Cluster_AlphaCapacity;
            this.Cluster_AlphaDistance = Cluster_AlphaDistance;
            this.Cluster_FitAlgoritm = Cluster_FitAlgoritm;
            this.Cluster_LS_Order = Cluster_LS_Order;
            this.Cluster_LS_SwapVehicle = Cluster_LS_SwapVehicle;
            this.Cluster_LS_InsertVehicle = Cluster_LS_InsertVehicle;
            this.Cluster_LS_SwapClusters = Cluster_LS_SwapClusters;
            //this.CluVRP_LS_SwapClusters = Cluster_LS_RndSwapVehicle;
            //this.CluVRP_LS_SwapVehicle = Cluster_LS_RndInsertVehicle;
            this.Cluster_LS_TwoOpt_Iterations = Cluster_LS_TwoOpt_Iterations;
            this.Cluster_LS_Relocate_Iterations = Cluster_LS_Relocate_Iterations;
            this.Cluster_LS_Exchange_Iterations = Cluster_LS_Exchange_Iterations;
            this.Customer_GRASPIterations = Customer_GRASPIterations;

            // Customer Level
            this.Customer_Alpha = Customer_Alpha;
            this.Customer_LS_Order = Customer_LS_Order;
            this.Customer_LS_SwapCustomers = Customer_LS_SwapCustomers;
            this.Customer_LS_TwoOpt_Iterations = Customer_LS_TwoOpt_Iterations;
            this.Customer_LS_Relocate_Iterations = Customer_LS_Relocate_Iterations;
            this.Customer_LS_Exchange_Iterations = Customer_LS_Exchange_Iterations;
        }

        // Class replicate json
        public class ParametersList
        {
            public CluVRPVersion[] CluVRP_Version { get; set; }
            public int[] CluVRP_GRASPIterations {get; set; }
            public int[] CluVRP_LS_Main_Iterations { get; set; }
            public int[] CluVRP_LS_SwapClusters { get; set; }
            public int[] CluVRP_LS_SwapVehicle { get; set; }
            public int[] Cluster_GRASPIterations { get; set; }
            public float[] Cluster_AlphaCapacity { get; set; }
            public float[] Cluster_AlphaDistance { get; set; }
            public int[] Cluster_FitAlgoritm { get; set; }
            public int[] Cluster_LS_Order { get; set; }
            public int[] Cluster_LS_SwapVehicle { get; set; }
            public int[] Cluster_LS_InsertVehicle { get; set; }
            public int[] Cluster_LS_SwapClusters { get; set; }
            //public int[] Cluster_LS_RndSwapVehicle { get; set; }
            //public int[] Cluster_LS_RndInsertVehicle { get; set; }
            public int[] Cluster_LS_TwoOpt_Iterations { get; set; }
            public int[] Cluster_LS_Relocate_Iterations { get; set; }
            public int[] Cluster_LS_Exchange_Iterations { get; set; }
            public int[] Customer_GRASPIterations { get; set; }
            public float[] Customer_Alpha { get; set; }
            public int[] Customer_LS_SwapCustomers { get; set; }
            public int[] Customer_LS_Order { get; set; }
            public int[] Customer_LS_TwoOpt_Iterations { get; set; }
            public int[] Customer_LS_Relocate_Iterations { get; set; }
            public int[] Customer_LS_Exchange_Iterations { get; set; }
        }
        
        // Read json object file and create all instances
        static public List<Parameters> parseParameterFile(string jsonFilePath)
        {
            List<ParametersList> list = loadJsonParameter(jsonFilePath);
            List<Parameters> ret = new List<Parameters>();
            
            foreach (ParametersList pInstance in list)
                foreach(CluVRPVersion cluvrp_version in pInstance.CluVRP_Version)
                    foreach (int cluvrp_GRASPIterations in pInstance.CluVRP_GRASPIterations)
                        foreach(int cluvrp_LS_Main_Iterarions in pInstance.CluVRP_LS_Main_Iterations)
                            foreach (int cluVRP_LS_SwapClusters in pInstance.CluVRP_LS_SwapClusters)
                                foreach (int cluVRP_LS_SwapVehicle in pInstance.CluVRP_LS_SwapVehicle)
                                    foreach (int cluster_GRASPIterations in pInstance.Cluster_GRASPIterations)
                                        foreach (double cluster_AlphaCapacity in pInstance.Cluster_AlphaCapacity)
                                            foreach (double cluster_AlphaDistance in pInstance.Cluster_AlphaDistance)
                                                foreach (FitAlgorithm cluster_FitAlgoritm in pInstance.Cluster_FitAlgoritm)
                                                    foreach (int cluster_LS_SwapVehicle in pInstance.Cluster_LS_SwapVehicle)
                                                        foreach (int cluster_LS_InsertVehicle in pInstance.Cluster_LS_InsertVehicle)
                                                            foreach (int cluster_LS_SwapClusters in pInstance.Cluster_LS_SwapClusters)
                                                                foreach (int cluster_LS_TwoOpt_Iterations in pInstance.Cluster_LS_TwoOpt_Iterations)
                                                                    foreach (int cluster_LS_Relocate_Iterations in pInstance.Cluster_LS_Relocate_Iterations)
                                                                        foreach (int cluster_LS_Exchange_Iterations in pInstance.Cluster_LS_Exchange_Iterations)
                                                                            foreach (int customer_GRASPIterations in pInstance.Customer_GRASPIterations)
                                                                                foreach (double customer_Alpha in pInstance.Customer_Alpha)
                                                                                     foreach(int customer_LS_SwapCustomers in pInstance.Customer_LS_SwapCustomers)
                                                                                        foreach (int customer_LS_TwoOpt_Iterations in pInstance.Customer_LS_TwoOpt_Iterations)
                                                                                            foreach (int customer_LS_Relocate_Iterations in pInstance.Customer_LS_Relocate_Iterations)
                                                                                                foreach (int customer_LS_Exchange_Iterations in pInstance.Customer_LS_Exchange_Iterations)
                                                                                                {
                                                                                                    // Create parameter instance
                                                                                                    Parameters parameter = new Parameters(
                                                                                                        cluvrp_version,
                                                                                                        cluvrp_GRASPIterations,
                                                                                                        cluvrp_LS_Main_Iterarions,
                                                                                                        cluVRP_LS_SwapClusters,
                                                                                                        cluVRP_LS_SwapVehicle,
                                                                                                        cluster_GRASPIterations,
                                                                                                        cluster_AlphaCapacity,
                                                                                                        cluster_AlphaDistance,
                                                                                                        cluster_FitAlgoritm,
                                                                                                        pInstance.Cluster_LS_Order,
                                                                                                        cluster_LS_SwapVehicle,
                                                                                                        cluster_LS_InsertVehicle,
                                                                                                        cluster_LS_SwapClusters,
                                                                                                        cluster_LS_TwoOpt_Iterations,
                                                                                                        cluster_LS_Relocate_Iterations,
                                                                                                        cluster_LS_Exchange_Iterations,
                                                                                                        customer_GRASPIterations,
                                                                                                        customer_Alpha,
                                                                                                        pInstance.Customer_LS_Order,
                                                                                                        customer_LS_SwapCustomers,
                                                                                                        customer_LS_TwoOpt_Iterations,
                                                                                                        customer_LS_Relocate_Iterations,
                                                                                                        customer_LS_Exchange_Iterations);

                                                                                                    // Add to list
                                                                                                    ret.Add(parameter);
                                                                                                }
            // Return parameter instance list
            return ret;
        }
        
        // Read Json file and make a parameters instance list
        static public List<ParametersList> loadJsonParameter(string jsonFilePath)
        {
            using (StreamReader r = new StreamReader(jsonFilePath))
            {
                string json = r.ReadToEnd();
                List<ParametersList> items = JsonConvert.DeserializeObject<List<ParametersList>>(json);
                return items;
            }
        }

    }
}
