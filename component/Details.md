
<iframe src="https://appetize.io/embed/287p205t6jfk8t7pu6am2wn08c?device=nexus5&scale=75&autoplay=true&orientation=portrait&deviceColor=black" 
        width="300px" height="597px" frameborder="0" scrolling="no"
        style="float:right;margin-left:1em;"></iframe>

**Jitter Physics** is a fast and lightweight physics engine for all managed 
languages.

**Platforms & Frameworks**
 - Every platform which supports .NET/Mono 
 - Works with the Mono framework on Linux/Mac without any recompilation 
 - No dependencies. Every 3D engine/framework is supported: OpenTK, SlimDX, 
   SharpDX, XNA, MonoGame, IrrlichtEngine 

**Overall Design** 
 - Written in pure C# with a clean and object orientated API 
 - Optimized for low to no garbage collections and maximum speed 
 - Supported Shapes: TriangleMesh, Terrain, Compound, MinkowskiSum, Box, Sphere, 
   Cylinder, Cone, Capsule, ConvexHull 
 - Take advantage of multi-core CPUs by using the internal multi-threading of 
   the engine 

## The Samples
_The samples use the [MonoGame][mg] framework and the MonoGame content 
pipeline. To build the samples, and the content, the MonoGame content
pipeline needs to be installed using the [MonoGame installer][mg-setup]._ 

## Quick Start

### Initialize the Physics System
Create a `World` instance and initialize it with a `CollisionSystem`:

    CollisionSystem collision = new CollisionSystemSAP();
    World world = new World(collision);

### Add Objects to the World
Create a shape of your choice, and pass it to a body:

    Shape shape = new BoxShape(1.0f, 2.0f, 3.0f);
    RigidBody body = new RigidBody(shape);

It is valid to use the same shape for different bodies. The 
position and orientation of the body can be set using it's properties.

The next step is to add the `Body` to the world:
 
    world.AddBody(body);
 
### Run the Simulation
Now you can call the `Step` method to integrate the world one timestep further. 
This should be done in you main game loop:
 
    while (gameRunning)
    {
        world.Step(1.0f / 100.0f, true);
        
        // do other stuff, like drawing
    }
 
The first parameter is the timestep. This value should be as small as possible 
to get a stable simulation. The second parameter is for whether using internal 
multi-threading or not. That's it the body is now simulated and affected by 
default gravity specified in `World.Gravity`. After each timestep the `Position` 
of the body should be different.

[mg]: http://www.monogame.net/
[mg-setup]: http://www.monogame.net/2015/04/29/monogame-3-4/
