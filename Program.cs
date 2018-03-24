using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CluVRP_GRASP
{
    class Program
    {
        static void Main(string[] args)
        {

            Logger logger = Logger.GetInstance();
            GVRPInstance[] instancias = InstanceParser.loadGVRPSetOfInstances("prueba");


        }
    }
        
}    
