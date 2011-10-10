using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter2D.Collision.Shapes;
using Jitter2D.LinearMath;
using Jitter2D.Dynamics;
using Microsoft.Xna.Framework;
using Jitter2D;
using Jitter2D.Dynamics.Constraints;

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
