using System;
using Jitter.Collision.Shapes;

namespace JitterDemo
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread()]
        static void Main(string[] args)
        {
            using (JitterDemo game = new JitterDemo())
            {
                game.Run();
            }
        }
    }
}

