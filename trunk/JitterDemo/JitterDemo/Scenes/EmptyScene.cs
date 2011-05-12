using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

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

        public override void Destroy()
        {
            RemoveGround();
        }
    }
}
