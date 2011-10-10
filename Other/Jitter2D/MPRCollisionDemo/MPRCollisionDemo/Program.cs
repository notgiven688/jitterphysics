using System;

namespace MPRCollisionDemo
{
#if WINDOWS || XBOX
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        static void Main(string[] args)
        {
            using (MPRCollisionDemo game = new MPRCollisionDemo())
            {
                game.Run();
            }
        }
    }
#endif
}

