using Godot;
using System;
using System.Linq;

public partial class player_controller : CharacterBody3D
{
    [ExportCategory("World")]
    [Export] Node3D WorldModel;
    [Export] CollisionShape3D CollisionShape;
    [ExportCategory("Head & Camera")]
    [Export] Node3D Head;
    [Export] Camera3D Camera;
    [Export] Node3D CameraSmooth;
    [Export] float mouse_look_sensitivity  = 0.006f;
    [Export] bool use_controller_look_smoothing  = true;
    [Export] float controller_look_sensitivity  = 0.05f;
    [Export] float controller_look_smoothing_value  = 5.0f;

    [ExportCategory("General Movement")]
    [Export] bool local_movement  = true;
    [Export] float jump_velocity  = 6.0f;
    [Export] bool auto_bhop  = true;
    [Export] bool auto_align_to_floor_normal  = false;
    [Export] float auto_align_rotation_speed = 10.0f;

    [ExportCategory("Ground Movement")]
    [Export] bool use_advanced_ground_movement  = true; //Basically use bhop
    [Export] float walk_speed  = 7.0f;
    [Export] float sprint_speed  = 8.5f;
    [Export] float ground_accel  = 14.0f;
    [Export] float ground_decel  = 10.0f;
    [Export] float ground_friction  = 6.0f;
    //Stairs
    [Export] RayCast3D StairsAheadRayCast3d ;
    [Export] RayCast3D StairsBelowRayCast3d ;
    [Export] float max_step_height = 0.5f;
    [Export] float min_step_height = 0.01f; //if running into issues lower this further (usually use 0 when using local movement)
    bool _snapped_to_stairs_last_frame  = false;
    float _last_frame_was_on_floor  = -1;
    //Ladder
    [Export] float climb_speed  = 7.0f;
    [Export] float ladder_jump_mult  = 0.5f;
    [Export] bool allow_ladder_boosting  = true;

    [ExportCategory("Air Movement")]
    [Export] bool use_air_movement  = true;
    [Export] bool use_surfing_movement  = true;
    [Export] float air_cap  = 0.85f; //the higher it is the steeper the ramps you can surf, makes it easier to stick and bhop
    [Export] float air_accel  = 800.0f;
    [Export] float air_move_speed  = 500.0f;
    [Export] Vector3 gravity_vector  = new Vector3(0,-1,0);
    [Export] float gravity_strength  = 12;

    [ExportCategory("Water Movement")]
    [Export] bool use_water_movement  = true;
    [Export] String water_area_group  = "water_area";
    [Export] float swim_up_speed  = 10;

    [ExportCategory("Crouch Movement")]
    [Export] float crouch_translate  = 0.7f;
    [Export] float crouch_speed_mult  = 0.8f;
    [Export] float crouch_cam_transition_smooth  = 7.0f;
    float CROUCH_JUMP_ADD{get => crouch_translate * 0.9f; set => CROUCH_JUMP_ADD = value;}
    float _original_capsule_height;
    bool is_crouched = false;

    [ExportCategory("Ridgidbody Interaction")]
    [Export] float my_approx_mass_kg  = 80.0f;
    [Export] float push_force_mult  = 5.0f;

    [ExportCategory("Headbob")]
    [Export] bool headbob_enabled  = true;
    [Export] float HEADBOB_MOVE_AMOUNT = 0.06f;
    [Export] float HEADBOB_FREQUENCY = 2.4f;
    float headbob_time  = 0.0f;

    [ExportCategory("Debug")]
    [Export] bool print_velocity_conversion  = false;
    [Export] bool allow_noclip  = true;
    [Export] float noclip_speed_mult  = 3.0f;
    float cur_noclip_speed_mult  = 3.0f;
    [Export] float noclip_sprint_speed_mult  = 3.0f;
    [Export] bool maintain_noclip_velocity  = true;
    bool noclip = false;

    Vector3 wish_dir = Vector3.Zero;
    Vector3 cam_aligned_wish_dir = Vector3.Zero;

    float get_move_speed()
    {
        if(is_crouched)
        {
            return walk_speed * crouch_speed_mult;
        }
        return Input.IsActionPressed("sprint") ? sprint_speed : walk_speed;
    }

    public override void _UnhandledInput(InputEvent @event)
    {
        base._UnhandledInput(@event);
        if(@event is InputEventMouseButton)
        {
            Input.MouseMode = Input.MouseModeEnum.Captured;
        }
        else if(@event.IsActionPressed("ui_cancel"))
        {
            Input.MouseMode = Input.MouseModeEnum.Visible;
        }

        if (Input.MouseMode == Input.MouseModeEnum.Captured)
        {
            if(@event is InputEventMouseMotion)
            {
                Rotate(Transform.Basis.Y.Normalized(),-((InputEventMouseMotion)@event).Relative.X * mouse_look_sensitivity); //Rotates around Local up
                Camera.RotateX(-((InputEventMouseMotion)@event).Relative.Y * mouse_look_sensitivity);
                Camera.Rotation = Camera.Rotation with { X = Mathf.Clamp(Camera.Rotation.X,float.DegreesToRadians(-90),float.DegreesToRadians(90))};
            }
        }

        //noclip speed mult
        if(@event is InputEventMouseButton && @event.IsPressed())
        {
            InputEventMouseButton MouseEvent = (InputEventMouseButton)@event;
            if(MouseEvent.ButtonIndex == MouseButton.WheelUp)
            {
                cur_noclip_speed_mult = Mathf.Min(100.0f,cur_noclip_speed_mult * 1.1f);
            }
            else if(MouseEvent.ButtonIndex == MouseButton.WheelDown)
            {
                cur_noclip_speed_mult = Mathf.Max(0.1f,cur_noclip_speed_mult * 0.9f);
            }
        }
    }

    public override void _Ready()
    {
        base._Ready();
        
        _original_capsule_height = ((CapsuleShape3D)CollisionShape.Shape).Height;

        foreach(VisualInstance3D child in WorldModel.FindChildren("*","VisualInstance3D"))
        {
            child.SetLayerMaskValue(1, false);
            child.SetLayerMaskValue(2, true);
        }
        Input.MouseMode = Input.MouseModeEnum.Captured;
    }

    public override void _Process(double delta)
    {
        base._Process(delta);
        _handle_controller_look_input((float)delta);
        if(IsOnFloor() && auto_align_to_floor_normal)
        {
            _handle_align_to_floor_normals((float)delta);
        }
    }

    public override void _PhysicsProcess(double delta)
    {
        base._PhysicsProcess(delta);
        float fDelta = (float)delta;

        if (IsOnFloor()) {_last_frame_was_on_floor = Engine.GetPhysicsFrames();}
        
        Vector2 input_dir = Input.GetVector("left","right","up","down").Normalized();
        if(local_movement)
        {
            wish_dir = new Vector3(input_dir.X,0,input_dir.Y); //When moving locally we rotate the velocity before moving so remove the rotation adjustments from here
        }   
        else
        {
            wish_dir = GlobalTransform.Basis * new Vector3(input_dir.X,0,input_dir.Y);
        }

        cam_aligned_wish_dir = Camera.GlobalTransform.Basis * new Vector3(input_dir.X,0,input_dir.Y);
        
        _handle_crouch(fDelta);
        
        if(!_handle_noclip(fDelta) && !_handle_ladder_physics(fDelta))
        {
            if (!_handle_water_physics(fDelta))
            {
                if (IsOnFloor() || _snapped_to_stairs_last_frame)
                {
                    _handle_ground_physics(fDelta);
                    if( Input.IsActionJustPressed("jump") || (auto_bhop && Input.IsActionPressed("jump")))
                    {
                        Vector3 local_up = GlobalTransform.Basis.Y;
                        Velocity += local_up * jump_velocity;
                    }
                        
                }
                else
                {
                    _handle_air_physics(fDelta);
                }  
                if(local_movement)
                {
                    Vector3 local_up = GlobalTransform.Basis.Y;
                    UpDirection = local_up;
                    //self.velocity = self.global_transform.basis * self.velocity
                }
                    
            }
        }
            
        bool snappedUp = false;
        snappedUp = _snap_up_stairs_check(fDelta);
        if(!snappedUp)
        {
            _push_away_ridgid_bodies();
            MoveAndSlide();
            _snap_down_to_stairs_check();
        }
            
        _slide_camera_smooth_back_to_origin(fDelta);
        
        if (print_velocity_conversion)
        {
            Vector3 global_vel = GlobalTransform.Basis.Inverse() * Velocity;
		    GD.Print("Local Velocity: ", Velocity, " -> Global Velocity: ", global_vel);
        }
            
    } 

    void _headbob_effect(float delta)
    {
        headbob_time += delta * Velocity.Length();
        Camera.Transform = Camera.Transform with {Origin = new Vector3(
            Mathf.Cos(headbob_time * HEADBOB_FREQUENCY * 0.5f) * HEADBOB_MOVE_AMOUNT,
            Mathf.Sin(headbob_time * HEADBOB_FREQUENCY) * HEADBOB_MOVE_AMOUNT,0)};
    }
	
	//Smoothly interpolated controller look with acceleration and deceleration
    Vector2 _cur_controller_look = new Vector2();
void _handle_controller_look_input(float delta)
    {
        Vector2 target_look = Input.GetVector("look_left","look_right","look_down","look_up").Normalized();

        if(use_controller_look_smoothing) //Smoothly interpolated controller look with acceleration and deceleration
        {
            if(target_look.Length() < _cur_controller_look.Length())
            {
                _cur_controller_look = target_look;
            } 
            else
            {
                _cur_controller_look = _cur_controller_look.Lerp(target_look,controller_look_smoothing_value * delta);
            }
        }
        else
        {
            _cur_controller_look = target_look;
        }

        Rotate(Transform.Basis.Y.Normalized(),-_cur_controller_look.X * controller_look_sensitivity); //Rotate around the local up
        Camera.RotateX(_cur_controller_look.Y * controller_look_sensitivity);
        Camera.Rotation = Camera.Rotation with { X = Mathf.Clamp(Camera.Rotation.X, Mathf.DegToRad(-90),Mathf.DegToRad(90)) };
    }

    Vector3 _saved_camera_global_pos = Vector3.Inf;
    void _save_camera_pos_for_smoothing()
    {
        if(_saved_camera_global_pos == Vector3.Inf)
        {
            _saved_camera_global_pos = CameraSmooth.GlobalPosition;
        }
        
    }
	
    void _slide_camera_smooth_back_to_origin(float delta)
    {
        if(_saved_camera_global_pos == Vector3.Inf){return;}

        Vector3 local_pos = CameraSmooth.Position; //Copy local pos so we can restore the X and Z
        CameraSmooth.GlobalPosition = _saved_camera_global_pos;
        CameraSmooth.Position = CameraSmooth.Position * Vector3.Up; //clears and x and Z so only the Y is set (probs not needed anymore)
        //Restore X and Z incase they were in use

        CameraSmooth.Position = CameraSmooth.Position with { X = local_pos.X};
        CameraSmooth.Position = CameraSmooth.Position with { Z = local_pos.Z};
        CameraSmooth.Position = CameraSmooth.Position with { Y = Mathf.Clamp(CameraSmooth.Position.Y, -0.7f, 0.7f)}; //Clamped incase teleported

        float move_amount = Mathf.Max(Velocity.Length() * delta, walk_speed/2 * delta);
        CameraSmooth.Position = CameraSmooth.Position with { Y = Mathf.MoveToward(CameraSmooth.Position.Y, 0.0f, move_amount)};
	    _saved_camera_global_pos = CameraSmooth.GlobalPosition;
	    if (CameraSmooth.Position.Y == 0)
        {
            _saved_camera_global_pos = Vector3.Inf; // Stop smoothing camera
        }
    }
	
	void _push_away_ridgid_bodies()
    {
        Vector3 local_up = local_movement ? GlobalTransform.Basis.Y : Vector3.Up;
        for (int i = 0; i < GetSlideCollisionCount(); i = i + 1) 
        {
            KinematicCollision3D c = GetSlideCollision(i);
            if (c.GetCollider() is RigidBody3D)
            {
                RigidBody3D cRigidBody = (RigidBody3D)c.GetCollider();
                Vector3 push_dir = -c.GetNormal();
                float velocity_diff_in_push_dir = Velocity.Dot(push_dir) - cRigidBody.LinearVelocity.Dot(push_dir);
                velocity_diff_in_push_dir = MathF.Max(0.0f,velocity_diff_in_push_dir); //Don't allow negative push force
                float mass_ratio = MathF.Min(1.0f,my_approx_mass_kg / cRigidBody.Mass);
                push_dir = push_dir - (push_dir * local_up); //prevents pushing objects above or below (less glitchy)
                float push_force = mass_ratio * push_force_mult;
                cRigidBody.ApplyImpulse(push_dir * velocity_diff_in_push_dir * push_force, c.GetPosition() - cRigidBody.GlobalPosition);
            }
        }
    }
	
    void _snap_down_to_stairs_check()
    {
        bool did_snap = false;
        bool floor_below = StairsBelowRayCast3d.IsColliding() && !is_surface_too_steep(StairsBelowRayCast3d.GetCollisionNormal());
        bool was_on_floor_last_frame = Engine.GetPhysicsFrames() - _last_frame_was_on_floor == 1;
        Vector3 global_vel = local_movement ? GlobalTransform.Basis.Inverse() * Velocity : Velocity; //Make sure velocity is in world coords
        if(!IsOnFloor() && global_vel.Y <= 0 && (was_on_floor_last_frame || _snapped_to_stairs_last_frame) && floor_below)
        {
            PhysicsTestMotionResult3D body_test_result = new PhysicsTestMotionResult3D();
            Vector3 local_up = local_movement ? GlobalTransform.Basis.Y : Vector3.Up;
            if (_run_body_test_motion(GlobalTransform,local_up * -max_step_height,body_test_result))
            {
                _save_camera_pos_for_smoothing();
                Vector3 global_travel = local_movement ? (GlobalTransform.Basis.Inverse() * body_test_result.GetTravel()) : (body_test_result.GetTravel()); //Make sure velocity is in world coords
                float translate_y = global_travel.Y;
                Position += translate_y * local_up;
                ApplyFloorSnap();
                did_snap = true;
            }
        }
        _snapped_to_stairs_last_frame = did_snap;
    }

    bool _snap_up_stairs_check(float delta)
    {
        if (!IsOnFloor() && !_snapped_to_stairs_last_frame){return false;}

        // Get the player's "up" direction
        Vector3 local_up = local_movement ? GlobalTransform.Basis.Y : Vector3.Up;
        // Compute horizontal velocity (ignoring vertical movement)
        Vector3 horizontal_velocity = Velocity - local_up * Velocity.Dot(local_up);
        // Don't snap stairs if the player is actively jumping/moving upwards significantly
        // or not moving forward enough
        float vertical_speed = Velocity.Project(local_up).Length();
        float min_horizontal_speed = 0.02f * Velocity.Length();
        if (vertical_speed > 0.2) {return false;}//Check player isn't jumping
        
        if (horizontal_velocity.Length() < min_horizontal_speed){return false;} //Check player is moving

        // Compute expected movement in the player's horizontal direction
        Vector3 expected_move_motion = horizontal_velocity * delta;

        // Calculate test position slightly above the expected move position
        Transform3D step_pos_with_clearance = GlobalTransform.Translated(expected_move_motion + local_up * (max_step_height * 2));
        // Run a test move downwards from this position to check for a step
        KinematicCollision3D down_check_result = new KinematicCollision3D();
        if (TestMove(step_pos_with_clearance, -local_up * (max_step_height * 2), down_check_result))
        {
            GodotObject collider = down_check_result.GetCollider();
            if (collider != null && (collider.IsClass("StaticBody3D") || collider.IsClass("CSGShape3D")))
            {
                // Calculate step height based on local up direction
                float step_height = (down_check_result.GetPosition() - GlobalPosition).Dot(local_up);
                // Step must be within valid height ranges
                if (step_height > max_step_height || step_height <= min_step_height) {return false;}
            
                // Position the StairsAhead raycast in front of the player
                StairsAheadRayCast3d.GlobalPosition = down_check_result.GetPosition() + local_up * max_step_height + expected_move_motion.Normalized() * 0.1f;
                StairsAheadRayCast3d.ForceRaycastUpdate();
                // Ensure there's space ahead and the surface isn't too steep
                if(StairsAheadRayCast3d.IsColliding() && !is_surface_too_steep(StairsAheadRayCast3d.GetCollisionNormal()))
                {
                    _save_camera_pos_for_smoothing();
                    GlobalPosition = step_pos_with_clearance.Origin + down_check_result.GetTravel();
                    ApplyFloorSnap();
                    _snapped_to_stairs_last_frame = true;
                    return true;
                }
                    
            }
        }
        return false;
    }

    void _handle_align_to_floor_normals(float delta)
    {
        Vector3 normal = StairsBelowRayCast3d.GetCollisionNormal();
        Vector3 forward = -GlobalTransform.Basis.Z;
        // Reconstruct the Basis with the new up direction
        Basis  target_basis = new Basis();
        target_basis.Y = normal;
        target_basis.X = forward.Cross(normal).Normalized();
        target_basis.Z = target_basis.X.Cross(normal).Normalized();
        Quaternion current_rotation = GlobalTransform.Basis.GetRotationQuaternion();
        Quaternion target_rotation = target_basis.GetRotationQuaternion();
        Quaternion smoothed_rotation = current_rotation.Slerp(target_rotation, auto_align_rotation_speed * delta);
        GlobalTransform = GlobalTransform with {Basis = new Basis(smoothed_rotation)};
    }

    Area3D _cur_ladder_climbing = null;
    bool _handle_ladder_physics(float delta)
    {
        bool was_climbing_ladder = _cur_ladder_climbing != null && _cur_ladder_climbing.OverlapsBody(this);
        if (!was_climbing_ladder)
        {
            _cur_ladder_climbing = null;
            foreach(Area3D ladder in GetTree().GetNodesInGroup("ladder_area3d"))
            {
                if(ladder.OverlapsArea(this))
                {
                    _cur_ladder_climbing = ladder;
                    break;
                }
                    
            }
                
            if(_cur_ladder_climbing == null){return false;}
        }
           
        
        Transform3D ladder_gtransform = _cur_ladder_climbing.GlobalTransform;
        Vector3 pos_rel_to_ladder = ladder_gtransform.AffineInverse() * GlobalPosition;
        
        float forward_move = Input.GetActionStrength("up") - Input.GetActionStrength("down");
        float side_move = Input.GetActionStrength("right") - Input.GetActionStrength("left");
        Vector3 ladder_forward_move = ladder_gtransform.AffineInverse().Basis * Camera.GlobalTransform.Basis * new Vector3(0,0,-forward_move);
        Vector3 ladder_side_move = ladder_gtransform.AffineInverse().Basis * Camera.GlobalTransform.Basis * new Vector3(side_move,0,0);
        float ladder_strafe_vel = climb_speed * (ladder_side_move.X + ladder_forward_move.X);
        
        float ladder_climb_vel = climb_speed * -ladder_side_move.Z;
        //bias direction based on angle looking up and down
        float up_wish = Vector3.Up.Rotated(new Vector3(1,0,0), Mathf.DegToRad(-45)).Dot(ladder_forward_move); //TODO: Might need to localize this later
        ladder_climb_vel += climb_speed * up_wish;
        
        //only mount ladders when moving towards them and stop sticking at the top
        bool should_dismount = false;
        if (!was_climbing_ladder)
        {
            bool mounting_from_top = pos_rel_to_ladder.Y > ((Node3D)_cur_ladder_climbing.GetNode("TopOfLadder")).Position.Y; //Ladder needs to have a TopOfLadder node3D
            if (mounting_from_top)
            {
                if (ladder_climb_vel > 0) {should_dismount = true;} //might clip the top but still try to leave
            }
            else
            {
                if((local_movement ? wish_dir : (ladder_gtransform.AffineInverse().Basis * wish_dir)).Z >= 0){ should_dismount = true;} //if falling through the air only attach if moving towards ladder
            }
                
            // Only stick if very close to ladder to prevent camera jittering and make it easier to get off
            if (MathF.Abs(pos_rel_to_ladder.Z) > 0.1f)
            {
                should_dismount = true;
            }
        }
            
        
        //let players step off at the floor
        if (IsOnFloor() && ladder_climb_vel <= 0){should_dismount = true;}

        if (should_dismount)
        {
            _cur_ladder_climbing = null;
            return false;
        }
            
        
        //Allow jumping off the ladder mid climb
        if (was_climbing_ladder && Input.IsActionJustPressed("jump"))
        {
            Velocity = _cur_ladder_climbing.GlobalBasis.Z * jump_velocity * ladder_jump_mult; //Jump away from the ladder with jump velocity * some multiplyer
            _cur_ladder_climbing = null;
            return false;
        }
            
        
        Velocity = ladder_gtransform.Basis * new Vector3(ladder_strafe_vel,ladder_climb_vel,0);
        if (!allow_ladder_boosting) //Stops players from exploiting diaganal movement to go flying
        {
            Velocity = Velocity.LimitLength(climb_speed);
        }
        
        //snap player to ladder
        pos_rel_to_ladder.Z = 0;
        GlobalPosition = ladder_gtransform * pos_rel_to_ladder;
        
        MoveAndSlide();
        return true;
        
    }

    bool _handle_water_physics(float delta)
    {
        if (GetTree().GetNodesInGroup(water_area_group).All(area => !((Area3D)area).OverlapsBody(this))){return false;}
		
        Vector3 local_up = local_movement ? GlobalTransform.Basis.Y : Vector3.Up;
        
        if (!IsOnFloor())
        {
            Velocity -= local_up * gravity_strength * 0.1f * delta;
        }
            
        
        Velocity += cam_aligned_wish_dir * get_move_speed() * delta;
        
        if (Input.IsActionPressed("jump"))
        {
            Velocity += local_up * swim_up_speed * delta;
        }
            
        
        Velocity = Velocity.Lerp(Vector3.Zero, 2 * delta); //Dampen velocity when hitting the water
        
        return true;
    }
	
    void _handle_crouch(float delta)
    {
        bool was_crouched_last_frame = is_crouched;
        Vector3 local_up = local_movement ? GlobalTransform.Basis.Y : Vector3.Up;
        if (Input.IsActionPressed("crouch"))
        {
            is_crouched = true;
        }    
        else if (is_crouched && !TestMove(GlobalTransform,local_up * crouch_translate))
        {
            is_crouched = false;
        }
            
        
        float translate_y_if_possible = 0.0f;
        if (was_crouched_last_frame != is_crouched && !IsOnFloor() && !_snapped_to_stairs_last_frame)
        {
            translate_y_if_possible = is_crouched ? CROUCH_JUMP_ADD : -CROUCH_JUMP_ADD;
        }
            
        
        if (translate_y_if_possible != 0.0)
        {
            KinematicCollision3D result = new KinematicCollision3D();
            TestMove(GlobalTransform,local_up * translate_y_if_possible,result);
            GlobalPosition += result.GetTravel();
            Head.GlobalPosition -= result.GetTravel();
            Head.Position = Head.Position with { Y = Mathf.Clamp(Head.Position.Y,-crouch_translate,0)};
        }
            
        Head.Position = Head.Position with { Y = Mathf.MoveToward(Head.Position.Y, is_crouched? -crouch_translate : 0,crouch_cam_transition_smooth * delta)};
        ((CapsuleShape3D)CollisionShape.Shape).Height = _original_capsule_height - (is_crouched ? crouch_translate : _original_capsule_height);
        CollisionShape.Position = CollisionShape.Position with { Y = ((CapsuleShape3D)CollisionShape.Shape).Height / 2};
    }

    bool _handle_noclip(float delta)
    {
        if (Input.IsActionJustPressed("_noclip") && OS.HasFeature("debug"))
        {
            noclip = !noclip;
            cur_noclip_speed_mult = noclip_speed_mult;
        }

        CollisionShape.Disabled = noclip;
        
        if (!noclip){return false;}
        
        float speed = get_move_speed() * cur_noclip_speed_mult;
        if(Input.IsActionJustPressed("sprint"))
        {
            speed *= noclip_sprint_speed_mult;
        }
        
        Velocity = cam_aligned_wish_dir * (maintain_noclip_velocity? speed : 0.0f);
        GlobalPosition += maintain_noclip_velocity ? (Velocity * delta) : (cam_aligned_wish_dir * speed * delta);
        return true;
    }

    void clip_velocity(Vector3 normal, float overbounce, float delta)
    {
        //When strafing into a wall, + gravity, velocity will be pointing roughly away from the normal
        //this code will back the players up and off the wall, cancelling out strafe + gravity, allowing surfing
        float backoff = Velocity.Dot(normal) * overbounce;
        if(backoff >= 0){return;}
        Vector3 change = normal * backoff;
        Velocity -= change;
        
        //Second iteration to ensure the player isn't stuck in the wall
        float adjust = Velocity.Dot(normal);
        if(adjust < 0.0)
        {
            Velocity -= normal * adjust;
        }
    }

    bool is_surface_too_steep(Vector3 normal)
    {
        Vector3 local_up = local_movement? GlobalTransform.Basis.Y : Vector3.Up;
	    return normal.AngleTo(local_up) > FloorMaxAngle;
    }
	
	bool _run_body_test_motion(Transform3D from, Vector3 motion, PhysicsTestMotionResult3D result = null)
    {
        if(result == null)
        {
            result = new PhysicsTestMotionResult3D();
        } 
        PhysicsTestMotionParameters3D testParams = new PhysicsTestMotionParameters3D();
        testParams.From = from;
        testParams.Motion = motion;
        return PhysicsServer3D.BodyTestMotion(GetRid(),testParams,result);
    }

    void _handle_air_physics(float delta) //Handles in air movement and surfing logic
    {
        Vector3 local_up = local_movement? GlobalTransform.Basis.Y : Vector3.Up;
	    Velocity -= local_up * gravity_strength * delta;

        //Souce/Quake feeling air movement
        if (use_air_movement)
        {
            float cur_speed_in_wish_dir = 0;
            Vector3 adjusted_wish_dir = local_movement ? (GlobalTransform.Basis * wish_dir) : wish_dir; //Used to ensure the calculations are made relative to players if needed
            cur_speed_in_wish_dir = Velocity.Dot(adjusted_wish_dir);
            float capped_speed = Mathf.Min((air_move_speed * wish_dir).Length(),air_cap);
            float add_speed_till_cap = capped_speed - cur_speed_in_wish_dir;
            if (add_speed_till_cap > 0)
            {
                float accel_speed = air_accel * air_move_speed * delta;
                accel_speed = Mathf.Min(accel_speed, add_speed_till_cap);
                Velocity += accel_speed * adjusted_wish_dir;
            }
        }

        if (use_surfing_movement && IsOnWall())
        {
            if (is_surface_too_steep(GetWallNormal()))
            {
                MotionMode = MotionModeEnum.Floating;
            }
            else
            {
                MotionMode = MotionModeEnum.Grounded;
            }
            clip_velocity(GetWallNormal(),1,delta); //Allows surfing
        }
    }

    void _handle_ground_physics(float delta)
    {
        if (use_advanced_ground_movement)//For Source/Quake like movement
        {
            float cur_speed_in_wish_dir = 0;
            Vector3 adjusted_wish_dir = local_movement? (GlobalTransform.Basis * wish_dir) : wish_dir; //Used to ensure the calculations are made relative to players if needed
            cur_speed_in_wish_dir = Velocity.Dot(adjusted_wish_dir);
            float add_speed_till_cap = get_move_speed() - cur_speed_in_wish_dir;
            if (add_speed_till_cap > 0)
            {
                float accel_speed = ground_accel * delta * get_move_speed();
                accel_speed = Mathf.Min(accel_speed, add_speed_till_cap);
                Velocity += accel_speed * adjusted_wish_dir;
            }
                
            //Apply Friction
            float control = Mathf.Max(Velocity.Length(),ground_decel);
            float drop = control * ground_friction * delta;
            float new_speed = Mathf.Max(Velocity.Length() - drop, 0.0f);
            if (Velocity.Length() > 0)
            {
                new_speed /= Velocity.Length();
            }
            Velocity *= new_speed;
        }  
        else
        {
            Velocity = Velocity with { X = wish_dir.X * get_move_speed()};
            Velocity = Velocity with { Z = wish_dir.Z * get_move_speed()};
            if (local_movement)
            {
                Velocity = Velocity with {Y = 0};
                Velocity = GlobalTransform.Basis * Velocity; //Rotate velocity vector from global to local coords
            }
                
        }
            
        if (headbob_enabled)
        {
          _headbob_effect(delta);
        } 
    }

}