extends Area3D

func _init():
	body_entered.connect(Entered)
	body_exited.connect(Exited)

func Entered(body : Node3D):
	var PlayerControllerComp = body.get_node_or_null("PlayeControllerComp")
	if PlayerControllerComp:
		PlayerControllerComp.auto_align_to_floor_normal = true

func Exited(body : Node3D):
	var PlayerControllerComp = body.get_node_or_null("PlayeControllerComp")
	if PlayerControllerComp:
		PlayerControllerComp.auto_align_to_floor_normal = false
