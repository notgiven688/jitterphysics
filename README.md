## Jitter Physics

[![JitterPhysics Build][appveyor-badge]][appveyor-link] [![JitterPhysics on NuGet][nuget-badge]][nuget-link]

Jitter Physics is a fast and lightweight 3D physics engine written in C#.

| Jitter Physics Cloth                  | Speculative Contacts Jitter Physics                  |
| :-----------------------------------: | :--------------------------------------------------: |
| [![Jitter Physics Cloth][img1]][vid1] | [![Speculative Contacts Jitter Physics][img2]][vid2] |


**Platforms & Frameworks**
 - Every platform which supports .NET, Mono or Xamarin
 - Works with the Mono framework on Linux/Mac without any recompilation 
 - Also supports the Xbox360 and Windows Phone _(up to v0.1.7)_
 - No dependencies. Every 3D engine/framework is supported: OpenTK, SlimDX, 
   SharpDX, XNA, IrrlichtEngine, Urho3D  

**Overall Design** 
 - Written in pure C# with a clean and object orientated API 
 - Optimized for low to no garbage collections and maximum speed 
 - Supported Shapes: TriangleMesh, Terrain, Compound, MinkowskiSum, Box, Sphere, 
   Cylinder, Cone, Capsule, ConvexHull 
 - Take advantage of multi-core CPUs by using the internal multithreading of 
   the engine 

## Jitter Physics (2D)

The ["Jitter-2D" branch][jitter2d] is still in development an is not complete.
At this time, it is probably better to make use of [Farseer Physics][farseer].

## Quick Start

### Initialize the Physics System
Create a world class and initialize it with a `CollisionSystem`:

    CollisionSystem collision = new CollisionSystemSAP();
    World world = new World(collision);

### Add Objects to the World
Create a shape of your choice and pass it to a body:

    Shape shape = new BoxShape(1.0f, 2.0f, 3.0f);
    RigidBody body = new RigidBody(shape);

It's valid to use the same shape for different bodies. 
Set the position and orientation of the body by using it's properties. 
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
multithreading or not. That's it the body is now simulated and affected by 
default gravity specified in `World.Gravity`. After each timestep the `Position` 
of the body should be different.

## Credits

This library was originally written by Thorben Linneweber. Original source code can be found at https://code.google.com/archive/p/jitterphysics.


[img1]: http://img.youtube.com/vi/cM23EJOFp3E/0.jpg
[vid1]: http://www.youtube.com/watch?v=
[img2]: http://img.youtube.com/vi/bKP2GZLlPWA/0.jpg
[vid2]: http://www.youtube.com/watch?v=bKP2GZLlPWA
[jitter2d]: https://github.com/mattleibow/jitterphysics/tree/Jitter-2D
[farseer]: https://farseerphysics.codeplex.com/

[appveyor-badge]: https://img.shields.io/appveyor/ci/mattleibow/JitterPhysics/master.svg?style=flat-square
[appveyor-link]: https://ci.appveyor.com/project/mattleibow/jitterphysics
[nuget-badge]: https://img.shields.io/nuget/v/JitterPhysics.svg?style=flat-square
[nuget-link]: https://www.nuget.org/packages/JitterPhysics/
