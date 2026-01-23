using Godot;
using Godot.Collections;
using System;

public partial class InteractionManager : Node
{
    [ExportCategory("Interaction")]
    [Export] ShapeCast3D InteractShapeCast3D;
    [Export] bool use_mouse_over_spherecast = false;
    [Export] float interaction_dist = 1000;
    bool doInteract;

    [Export] Camera3D Camera;

    InteractionComponent current_interactable = null;

    public override void _Process(double delta)
    {
        base._Process(delta);
        InteractionComponent target_interactable = try_get_interactable_component();
        if (target_interactable != current_interactable)
        {
            if (current_interactable != null)
            {
                current_interactable.hover_stopped(this);
            }
            current_interactable = target_interactable;
            if(current_interactable != null)
            {
                current_interactable.hover_started(this);
            }
                
        }
            
        if (Input.IsActionJustPressed("interact"))
        {
            if (current_interactable != null)
            {
                current_interactable.interact();
            }
        } 
            
    }

    InteractionComponent try_get_interactable_component()
    {
        if (use_mouse_over_spherecast)
        {
            Vector2 mouse = GetViewport().GetMousePosition();
            PhysicsDirectSpaceState3D worldSpace = Camera.GetWorld3D().DirectSpaceState;
            Vector3 start = Camera.ProjectRayOrigin(mouse);
            Vector3 end = Camera.ProjectPosition(mouse, 1000);
            Dictionary results = worldSpace.IntersectRay(PhysicsRayQueryParameters3D.Create(start,end));
            
            if (results.ContainsKey("collider"))
            {
                if (((Node)results["collider"]).GetNodeOrNull("InteractableComponent") is InteractionComponent)
                {
                    InteractionComponent IComp =  (InteractionComponent)((Node)results["collider"]).GetNodeOrNull("InteractableComponent");
                    return IComp;
                }
                    
            }
            return null;
        }
        else
        {
            for (int i = 0; i < InteractShapeCast3D.GetCollisionCount(); i = i + 1)
            {
                if (i > 0 && InteractShapeCast3D.GetCollider(0) != this){return null;}
                if (((Node)InteractShapeCast3D.GetCollider(i)).GetNodeOrNull("InteractableComponent") is InteractionComponent)
                {
                    InteractionComponent IComp =  (InteractionComponent)((Node)InteractShapeCast3D.GetCollider(i)).GetNodeOrNull("InteractableComponent");
                    return IComp;
                }
            }
        }
        return null;
    }
	
	

}
