using Godot;
using System;

public partial class InteractionComponent : Node
{
    InteractionManager currentImanager = null;

    [Signal] public delegate void OnInteractedEventHandler();
    [Signal] public delegate void OnHoverStartedEventHandler();
    [Signal] public delegate void OnHoverEndedEventHandler();
    [Signal] public delegate void OnHoverUpdateEventHandler();

    public void interact()
    {
        GD.Print(GetParent().Name + ": Interacted");
	    EmitSignal(SignalName.OnInteracted);
    }

    public void hover_started(InteractionManager Imanager)
    {
        GD.Print(GetParent().Name + ": Hover Started");
	    if(currentImanager != null)
        {
            currentImanager = Imanager;
            EmitSignal(SignalName.OnHoverStarted);
        }
            
    }
	

    public void hover_stopped(InteractionManager Imanager)
    {
        if(Imanager == currentImanager)
        {
            currentImanager = null;
            GD.Print(GetParent().Name + ": Hover Ended");
            EmitSignal(SignalName.OnHoverEnded);
        }
        
    }

    public override void _Process(double delta)
    {
        base._Process(delta);
        if(currentImanager != null)
        {
            EmitSignal(SignalName.OnHoverUpdate);
        }
    }

}
