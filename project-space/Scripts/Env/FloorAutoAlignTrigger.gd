extends Area3D

var playerCSharpScript = load("res://Scripts/Player/player_controller.cs")
var playernode : CharacterBody3D = playerCSharpScript.new()

func _init():
	body_entered.connect(Entered)
	body_exited.connect(Exited)

func Entered(body : Node3D):
	pass
	#if body is playernode:
	#	body.auto_align_to_floor_normal = true

func Exited(body : Node3D):
	pass
	#if body is playernode:
	#	body.auto_align_to_floor_normal = false
