using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter.Collision.Shapes;
using Jitter.LinearMath;
using Jitter.Dynamics;
using Microsoft.Xna.Framework;
using Jitter;
using Jitter.Dynamics.Constraints;

namespace JitterDemo.Scenes
{

    public class EmptyScene : Scene
    {
       
        public EmptyScene(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();
        }
    }


}
