using Godot;
using Godot.Collections;
using System;
using System.Runtime.CompilerServices;

public partial class WorldObjectManagerComponent : Node
{
    
    public WorldObjectManagerComponent ParentWorldManager; //If null then player is in real world

    public Array<WorldObjectManagerComponent> nestedObjects = new Array<WorldObjectManagerComponent>(); 

    int ExternalPhysicsLayer = 10;

    int InternalPhysicsLayer = 11;

    [Export] public Node3D WorldVisuals;

    [Export] public PhysicsBody3D WorldExternalPhysics;

    [Export] public PhysicsBody3D WorldInternalPhysics; //Mainly used for ships and stations

    public override void _Ready()
    {
        base._Ready();
        
        WorldsManager.Instance.RegisterWorld(this);

        UpdateCollisionLayers(WorldExternalPhysics,false);

        if(WorldInternalPhysics != null)
        {
            UpdateCollisionLayers(WorldInternalPhysics,true);
        }
    }


    public void EnterWorld(WorldObjectManagerComponent objectManager)
    {
        if(WorldInternalPhysics == null){return;} //Don't enter a world without internals
        if(nestedObjects.Contains(objectManager)){return;}
        if(objectManager.ParentWorldManager == this){return;}
        
        Node3D EnteringWorldVisuals = objectManager.WorldVisuals;
        PhysicsBody3D EnteringWorldExternalPhysics = objectManager.WorldExternalPhysics;
        PhysicsBody3D EnteringWorldInternalPhysics = objectManager.WorldInternalPhysics;

        nestedObjects.Add(objectManager);

        if(objectManager.ParentWorldManager != null)
        {
            objectManager.ParentWorldManager.nestedObjects.Remove(objectManager);
        }
        objectManager.ParentWorldManager = this;

        Vector3 localVelocity = Vector3.Zero;
		Vector3 localAngularVelocity = Vector3.Zero;
        RigidBody3D EnteringWorldRB = null;
        CharacterBody3D EnteringWorldChar = null;
        if(EnteringWorldExternalPhysics is RigidBody3D)
        {
            EnteringWorldRB = (RigidBody3D)EnteringWorldExternalPhysics;
            Vector3 LocalEntryVelocity = WorldExternalPhysics.GlobalTransform.Basis.Inverse() * EnteringWorldRB.LinearVelocity;
            localVelocity = WorldInternalPhysics.GlobalTransform.Basis * LocalEntryVelocity;

            Vector3 LocalEntryAngularVelocity = WorldExternalPhysics.GlobalTransform.Basis.Inverse() * EnteringWorldRB.LinearVelocity;
			localAngularVelocity = WorldInternalPhysics.GlobalTransform.Basis * LocalEntryAngularVelocity;
        }
        if(EnteringWorldExternalPhysics is CharacterBody3D)
        {
            EnteringWorldChar = (CharacterBody3D)EnteringWorldExternalPhysics;
            Vector3 LocalEntryVelocity = WorldExternalPhysics.GlobalTransform.Basis.Inverse() * EnteringWorldChar.Velocity;
            localVelocity = WorldInternalPhysics.GlobalTransform.Basis * LocalEntryVelocity;
        }

        Vector3 localPos = WorldVisuals.ToLocal(EnteringWorldVisuals.GlobalPosition); //Find the offset of relative to the 
        
        ReparentNodes(EnteringWorldVisuals,WorldVisuals);
		Vector3 localRot = EnteringWorldVisuals.Rotation;

        //Sort physics transform
        ReparentNodes(EnteringWorldExternalPhysics,WorldInternalPhysics);

        EnteringWorldExternalPhysics.Position = localPos;
        EnteringWorldExternalPhysics.Rotation = localRot;

        if(EnteringWorldRB != null)
        {
            EnteringWorldRB.LinearVelocity = localVelocity;
            EnteringWorldRB.AngularVelocity = localAngularVelocity;
        }
        if(EnteringWorldChar != null)
        {
            EnteringWorldChar.Velocity = localVelocity;
        }

        objectManager.ParentWorldManager = this;
        
        UpdateCollisionLayers(EnteringWorldExternalPhysics,true);
        

        GD.Print("Entered object");
    }

    public void ExitWorld(WorldObjectManagerComponent objectManager)
    {
        if(!nestedObjects.Contains(objectManager)){return;}
        if(objectManager.ParentWorldManager != this){return;}

        Node3D ExitingWorldVisuals = objectManager.WorldVisuals;
        PhysicsBody3D ExitingWorldExternalPhysics = objectManager.WorldExternalPhysics;
        PhysicsBody3D ExitingWorldInternalPhysics = objectManager.WorldInternalPhysics;

        nestedObjects.Remove(objectManager);

        if(ParentWorldManager != null)
        {
            ParentWorldManager.EnterWorld(objectManager);
        }

        Vector3 localVelocity = Vector3.Zero;
		Vector3 localAngularVelocity = Vector3.Zero;
        RigidBody3D ExitingWorldRB = null;
        CharacterBody3D ExitingWorldChar = null;
        
        
         
        Vector3 VisualPosition = ExitingWorldVisuals.GlobalPosition;
        Vector3 visualRotation = ExitingWorldVisuals.GlobalRotation;
        ReparentNodes(ExitingWorldVisuals,WorldsManager.Instance);
        //Sort physics transform

        if(ExitingWorldExternalPhysics is RigidBody3D)
        {
            ExitingWorldRB = (RigidBody3D)ExitingWorldExternalPhysics;
            Vector3 LocalEntryVelocity = WorldInternalPhysics.GlobalTransform.Basis.Inverse() * ExitingWorldRB.LinearVelocity;
            localVelocity = WorldExternalPhysics.GlobalTransform.Basis * LocalEntryVelocity;
            Vector3 LocalEntryAngularVelocity = WorldInternalPhysics.GlobalTransform.Basis.Inverse() * ExitingWorldRB.LinearVelocity;
			localAngularVelocity = WorldExternalPhysics.GlobalTransform.Basis * LocalEntryAngularVelocity;
        }
        if(ExitingWorldExternalPhysics is CharacterBody3D)
        {
            ExitingWorldChar = (CharacterBody3D)ExitingWorldExternalPhysics;
            Vector3 LocalEntryVelocity = WorldInternalPhysics.GlobalTransform.Basis.Inverse() * ExitingWorldChar.Velocity;
            localVelocity = WorldExternalPhysics.GlobalTransform.Basis * LocalEntryVelocity;
        }
        ReparentNodes(ExitingWorldExternalPhysics,WorldsManager.Instance);

        ExitingWorldExternalPhysics.GlobalPosition = VisualPosition;
        ExitingWorldExternalPhysics.GlobalRotation = visualRotation;

        UpdateCollisionLayers(ExitingWorldExternalPhysics,false);
        
        if(ExitingWorldRB != null)
        {
            ExitingWorldRB.LinearVelocity = localVelocity;
            ExitingWorldRB.AngularVelocity = localAngularVelocity;
        }
        if(ExitingWorldChar != null)
        {
            ExitingWorldChar.Velocity = localVelocity;
        }

        objectManager.ParentWorldManager = null;
        GD.Print("Exited object");
        
    }

    void ReparentNodes(Node3D Child, Node newParent)
    {
        if (Child.GetParent() != null) //need to remove all parents before adding as a new child (weird godot stuff)
        {
            Child.GetParent().RemoveChild(Child);
            newParent.AddChild(Child);
            //Child.Reparent(newParent);
        }
        else
        {
            newParent.AddChild(Child);
        }
    }

    void UpdateCollisionLayers(CollisionObject3D Col, bool isEntering)
    {
        Col.SetCollisionLayerValue(1,!isEntering);
        Col.SetCollisionLayerValue(ExternalPhysicsLayer,!isEntering);
        Col.SetCollisionLayerValue(InternalPhysicsLayer,isEntering);

        Col.SetCollisionMaskValue(1,!isEntering);
        Col.SetCollisionMaskValue(ExternalPhysicsLayer,!isEntering);
        Col.SetCollisionMaskValue(InternalPhysicsLayer,isEntering);

        foreach(Node childNode in Col.GetChildren())
        {
            if(childNode is CollisionObject3D)
            {
                UpdateCollisionLayers((CollisionObject3D)childNode,isEntering);
            }
        }
    }

    public override void _Process(double delta)
    {
        base._Process(delta);

        WorldVisuals.Position = WorldExternalPhysics.Position;
        WorldVisuals.Rotation = WorldExternalPhysics.Rotation;
    }


}
