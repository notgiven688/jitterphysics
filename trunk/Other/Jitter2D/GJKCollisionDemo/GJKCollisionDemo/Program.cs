using System;

namespace GJKCollisionDemo
{
#if WINDOWS || XBOX
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        static void Main(string[] args)
        {
            using (GJKCollisionDemo game = new GJKCollisionDemo())
            {
                game.Run();
            }
        }
    }
#endif
}

