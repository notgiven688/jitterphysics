## Jitter Physics (2D)

_Ths branch is still in development and is not complete.
At this time, it is probably better to make use of [Farseer Physics][farseer]._

Jitter Physics (2D) is a fast and lightweight 2D physics engine written in C#. 
It has been branched from the main 3D engine for further 2D optimisations. 

**Platforms & Frameworks**
 - Every platform which supports .NET/Mono 
 - Works with the Mono framework on Linux/Mac without any recompilation 
 - Also supports the XBox360 and the WindowsPhone _(up to v0.1.7)_
 - No dependencies.

**Overall Design** 
 - Written in pure C# with a clean and object orientated API 
 - Optimized for low to no garbage collections and maximum speed 
 - Take advantage of multi-core CPUs by using the internal multithreading of 
   the engine 

## Jitter Physics (3D)

The ["master", or Jitter 3D, branch][jitter3d] is the main 3D vesion of the Jitter
physics engine.

[jitter3d]: https://github.com/mattleibow/jitterphysics/tree/master
[farseer]: https://farseerphysics.codeplex.com/
