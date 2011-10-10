using System;

namespace JitterDemo
{
#if WINDOWS || XBOX
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        static void Main(string[] args)
        {
            using (JitterDemo game = new JitterDemo())
            {
                game.Run();
            }
        }
    }
#endif
}

