using Godot;
using Godot.Collections;
using System;

public partial class PhysicsWorldSwitchArea : Area3D
{
    [Export] public WorldObjectManagerComponent LinkedWorldManager;
    [Export] public Array<PhysicsBody3D> BodiesToIgnore;

    [Export] public bool IsEntryDoor;
    public override void _Ready()
    {
        base._Ready();
        if(IsEntryDoor)
        {
            BodyEntered += OnEntered;
        }
        else
        {
            BodyExited += OnExited;
        }
    }

    void OnEntered(Node3D body)
    {
        
        WorldObjectManagerComponent TransitingWorld = null;
        if(body is PhysicsBody3D)
        {
            if(WorldsManager.Instance.PhysicsWorlds.ContainsKey((PhysicsBody3D)body))
            {
                if(WorldsManager.Instance.PhysicsWorlds[(PhysicsBody3D)body] == LinkedWorldManager){return;}
                TransitingWorld = WorldsManager.Instance.PhysicsWorlds[(PhysicsBody3D)body];
            }
            else
            {
                return;
            }
        }
        if(TransitingWorld == null){return;}

        Entered(TransitingWorld);
    }
    void OnExited(Node3D body)
    {
        WorldObjectManagerComponent TransitingWorld = null;
        if(body is PhysicsBody3D)
        {
            if(WorldsManager.Instance.PhysicsWorlds.ContainsKey((PhysicsBody3D)body))
            {
                if(WorldsManager.Instance.PhysicsWorlds[(PhysicsBody3D)body] == LinkedWorldManager){return;}
                TransitingWorld = WorldsManager.Instance.PhysicsWorlds[(PhysicsBody3D)body];
            }
            else
            {
                return;
            }
        }
        if(TransitingWorld == null){return;}

        Exited(TransitingWorld);
        // if ((Basis * Vector3.Forward).Dot(body.Position - Position) > 0) //get the dot product
		// {
			
		// }
		// else
		// {
			
		// }
    }

    

    private void Entered(WorldObjectManagerComponent TransitingWorld)
	{
		if (LinkedWorldManager != null)
		{
			if (TransitingWorld.ParentWorldManager != LinkedWorldManager)
			{
				LinkedWorldManager.EnterWorld(TransitingWorld);
			}
		}
        else
        {
            GD.PrintErr(this + " No physics world present");
        }
	}

	private void Exited(WorldObjectManagerComponent TransitingWorld)
	{
		
		if (LinkedWorldManager != null)
		{
			if (TransitingWorld.ParentWorldManager == LinkedWorldManager)
			{
				LinkedWorldManager.ExitWorld(TransitingWorld);
			}
			else
			{
				GD.PrintErr(TransitingWorld + " Physics world mismatch on exit");
			}
			GD.Print("Exited");
		}
		else
		{
			GD.PrintErr(this + " No physics world present");
		}
	}

}
