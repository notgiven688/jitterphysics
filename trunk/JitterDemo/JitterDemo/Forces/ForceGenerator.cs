using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Jitter.Forces
{

    /// <summary>
    /// Base class for physic effect.
    /// </summary>
    public class ForceGenerator
    {

        /// <summary>
        /// 
        /// </summary>
        protected World world;

        private WorldStep preStep, postStep;

        /// <summary>
        /// 
        /// </summary>
        /// <param name="world"></param>
        public ForceGenerator(World world)
        {
            this.world = world;

            preStep = new WorldStep(PreStep);
            postStep = new WorldStep(PostStep);

            world.PostStep += postStep;
            world.PreStep += preStep;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="timeStep"></param>
        public virtual void PreStep(float timeStep)
        {
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="timeStep"></param>
        public virtual void PostStep(float timeStep)
        {
        }

        /// <summary>
        /// 
        /// </summary>
        public void RemoveEffect()
        {
            world.PostStep -= postStep;
            world.PreStep -= preStep;
        }


    }
}
