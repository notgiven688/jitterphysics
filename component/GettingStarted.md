## The World
The first step in setting up the engine in your game is to create a new instance 
of the `World` class:
 
    CollisionSystem collisionSystem = new CollisionSystemSAP();
    World world = new World(collisionSystem);

The world instance is created by passing a `CollisionSystem` class to the 
constructor. In this case we used `CollisionSystemSAP`. SAP stands for "Sweep and
Prune" which is how the broadphase collision works. There is also a 
`CollisionSystem` called `CollisionSystemBrute` which just uses brute force for 
the broadphase system - which is pretty fast in smaller scenes. From now on the 
`World` instance uses the chosen `CollisionSystem` to detect collisions. It's 
possible (and also a good idea) to use Jitter for collision detection only. You 
just don't create the World class which handles the 'dynamics' and use the 
methods provides by the `CollisionSystem`.

## The CollisionSystem
After adding the `CollisionSystem` to the world, the `CollisionSystem` gets 
automatically updated. Adding/Removing a `RigidBody` to/from the `World` also 
adds/removes it to/from the `CollisionSystem`. The `World` class internally calls 
`CollisionSystem.Detect` to get all collisions between all bodies. If you want 
to use Jitter just for collisions the following code detecting collisions 
between all bodies and reporting them back is interesting for you:
 
    private DetectTest()
    {
        CollisionSystem collisionSystem = new CollisionSystemSAP();
        collisionSystem.CollisionDetected += OnCollisionDetected;
        
        // Here some bodies have to be added to the collisionSystem.
        collisionSystem.Detect(true);
    }
    
    private void CollisionDetected(
        RigidBody body1, RigidBody body2, 
        JVector point1, JVector point2, 
        JVector normal, 
        float penetration)
    {
        System.Diagnostics.Debug.WriteLine("Collision detected!");
    }
 
The method `CollisionDetected` is called when there is a collision. It's also 
possible to check just two bodies against each other:
 
    collisionSystem.Detect(rigidBody1, rigidBody2);
 
If there is a listener of the `CollisionDetected` event and there is a collision 
the event gets called providing you with several details about the collision. If 
you don't want to use this functionality you just don't use it. The world class 
does the collision detection internally.

An interesting feature of the engine - no matter if you use just the collision 
or also the dynamics - is to raycast the entire scene, containing all your 
bodies:
 
    private DetectTest()
    {
        RigidBody resBody;
        JVector hitNormal:
        float fraction;
        bool result;
        
        bool result = world.CollisionSystem.Raycast(
            JVector.One, 
            JVector.Right * 100.0f,
            RaycastCallback, 
            out resBody, 
            out hitNormal, 
            out fraction);
        
        if (result) 
        {
            System.Diagnostics.Debug.Writeln("Collision detected!");
        }
    }
    
    private bool RaycastCallback(RigidBody body, JVector normal, float fraction)
    {
        return !body.IsStatic;
    }
 
Raycasting means that a ray (a line with a start and an direction in 
3-dimensional space. here: (1,1,1) and (-100,0,0)) is 'shot' from it's start 
along it's direction. The ray 'hits' one of the bodies within the scene and 
the collision information is reported back. 'fraction' gives you the 
information where (at the ray) the collision occurred:
 
    JVector hitPoint = JVector.One + fraction * (JVector.Right * 100.0 f);
 
The `RaycastCallback` gives you the possibility to decide if a body should be 
considered as a collision object for the ray. In the sample above static bodies 
aren't considered. The ray goes right 'through' them. You can also pass `null` 
as a parameter for the `RaycastCallback` - so every body is considered. 
Raycasting is one of the core functionalities of a physic engine and has many 
applications: consider for example, you want to implement a gun in your game and 
apply a force to the object the player hit.

Jitter also gives you the possibility to use core detection functions. These are 
the static classes `XenoCollide` and `GJKCollide`. You can use their `Detect` 
functions to check for collisions of shapes which are defined by their 
`ISupportMap` implementation. This is for more advanced users.

## Adding Bodies
So far we just have created our `World` class which is connected to the 
`CollisionSystem` - our world is pretty empty and we want to add a single box to 
it:
 
    CollisionSystem collisionSystem = new CollisionSystemSAP();
    World world = new World(collisionSystem);
    
    Shape shape = new BoxShape(JVector.One);
    RigidBody body = new RigidBody(shape);
    
    world.AddBody(body);
 
First a `Shape` is created. The shape represents the collidable part of the 
`RigidBody`. We wanted to have a box, so we used the `BoxShape`. There are much 
more default shapes in Jitter: `Box`, `Cone`, `Sphere`, `Cylinder`, `Capsule`, 
`Compound`, `TriangleMesh`. The shape is passed to the constructor of the body 
which gets added to the world. Done. Calculating mass, inertia.. is done inside 
the engine - but you also can set it manually. The body with the side length 
(1,1,1) is created at the origin (with `RigidBody.Position` you set the position
of the center of mass of the simple shapes).

 > Units in Jitter: The default gravity which adds force to a body is set to 
 > 9.81 m/s². So, one length unit in Jitter is one meter. Because of numerical 
 > instabilities when using smaller objects it's recommended not to use objects 
 > which are much smaller than one unit.

The last thing to do, is to update the engine and to integrate the current 
physics state a timestep further:
 
    world.Step(1.0f / 100.0f, true);
 
The first parameter is the timestep, the second one tells the engine to use 
internal multi-threading or not. In our simple one-body world the box gets 
affected by gravity and the position changes. In your 'Draw' method you are now 
able to draw a box with the position and orientation of the body.

## Complete Sample
Here, we create a complete physics scene, and simulated using just a few 
lines of code. 
 
    private void BuildScene()
    {
        // creating two box shapes with different sizes
        Shape boxShape = new BoxShape(JVector.One);
        Shape groundShape = new BoxShape(new JVector(10, 1, 10));
        
        // create new instances of the rigid body class and pass
        // the boxShapes to them
        RigidBody boxBody1 = new RigidBody(boxShape);
        RigidBody boxBody2 = new RigidBody(boxShape);
        RigidBody groundBody = new RigidBody(groundShape);
        
        // set the position of the box size=(1,1,1)
        // 2 and 5 units above the ground box size=(10,1,10)
        boxBody1.Position = new JVector(0, 2, 0.0f);
        boxBody2.Position = new JVector(0, 5, 0.3f);
        
        // make the body static, so it can't be moved
        groundBody.IsStatic = true;
        
        // add the bodies to the world.
        world.AddBody(boxBody1);
        world.AddBody(boxBody2);
        world.AddBody(groundBody);
    }
 
The scene is drawn with the MonoGame framework, but it would be similar for 
every other graphics framework/engine. Jitter uses it's own math classes 
(`JVector`, `JMatrix`, `JBox`, `JMath`, `JOctree`), so you have to convert 
orientations (represented by 3x3 matrices) and positions to the type of your 
graphics framework this isn't that complicated. For MonoGame the conversion 
method looks like this:
 
    public static Matrix ToMatrix(JMatrix matrix)
    {
        return new Matrix(
            matrix.M11, matrix.M12, matrix.M13, 0.0f,
            matrix.M21, matrix.M22, matrix.M23, 0.0f,
            matrix.M31, matrix.M32, matrix.M33, 0.0f, 
            0.0f, 0.0f, 0.0f, 1.0f);
    }
 
Building a MonoGame (4x4) world matrix to draw your object is easy:
 
    Matrix matrix = Conversion.ToMatrix(body.Orientation);
    matrix.Translation = Conversion.ToVector(body.Position);
