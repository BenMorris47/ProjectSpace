using Godot;
using System;
using System.Collections.Generic;

public partial class WorldsManager : Node
{
    public static WorldsManager Instance;

    Node3D ExternalPhysicsContainer;

    public Dictionary<Node3D,WorldObjectManagerComponent> VisualWorlds = new Dictionary<Node3D,WorldObjectManagerComponent>();

    public Dictionary<PhysicsBody3D,WorldObjectManagerComponent> PhysicsWorlds = new Dictionary<PhysicsBody3D,WorldObjectManagerComponent>();

    public override void _Ready()
    {
        base._Ready();
        Instance ??= this; // Set as this if null
        if(Instance != this) //Ensure only the first instance exists
        {
            QueueFree();
        }
        else
        {
            ExternalPhysicsContainer = new Node3D();
            AddChild(ExternalPhysicsContainer);
        }
        
    }

    public void RegisterWorld(WorldObjectManagerComponent World)
    {
        VisualWorlds.Add(World.WorldVisuals,World);
        PhysicsWorlds.Add(World.WorldExternalPhysics,World);
    }
}
