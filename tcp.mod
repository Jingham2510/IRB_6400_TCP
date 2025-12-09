MODULE tcp

    
    
    !Variables to store socket info and socket commands
    VAR socketdev server_socket;
    VAR socketdev client_socket;
    VAR string receive_string;
    VAR string client_ip;
    
    
     !Spherical end effector tool
     PERS tooldata sph_end_eff := [TRUE, [[0, 0, 100], [1, 0, 0, 0]],
                        [1.0687, [0, 0, 42.664],[1, 0, 0, 0], 0, 0, 0]];
    
    
    VAR loaddata test_load := [0.001, [0, 0, 0.001],[1, 0, 0, 0], 0, 0, 0];
    
    
    
    VAR fcforcevector force_vector;
    
    


    !If force control has been calibrated
    VAR bool fc_cal := FALSE;
    
    !Force control flags/values
    VAR bool fc_mode := FALSE;
    
    VAR string force_ax := "na";
    VAR num force_target := 0;
     
    !Initialise Trajectory queue
    VAR robtarget traj_coord_queue{100000};    
    VAR num rel_move_queue{100000, 3};
    
    VAR num head := 1;
    VAR num tail := 1;
    
    !Tracker of most recent trajectory pnt
    VAR num lst_trans_pnt := -1;
    VAR num lst_rot_pnt := -1;
    
    !Go/Pause Flag
    VAR bool run_trajectory := FALSE;
    VAR num conc_count := 0;
    
    !traj done and still count
    VAR bool traj_done := FALSE;
    VAR num still_cnt := 0;
    
    
    !Empty Flag
    VAR bool queue_end := TRUE;  
    
    !ROBOT VARIABLES
    VAR num des_speed_num := 50;
    VAR speeddata des_speed := [50, 500, 5000, 1000];
    
    !Force compensation values
    VAR bool fm_compensate := FALSE;
    VAR num comp_dist := 0;

    VAR bool vert_force_found := FALSE;

    PROC main()      
        



        SetDo move_started, 0;
        
        IF TRUE THEN
           go_home;
           
        ENDIF
        
        !Checks if force calibraiton is required or not
        !Not required for the virtual controller
        IF ROBOS() AND (NOT fc_cal) THEN                
     
            !Calibrate the load sensor - the documentation reccomends making a fine movement before the calibration   
            
            MoveL RelTool( CRobT(\Tool:=sph_end_eff \WObj:=wobj0), 0, 0, -10), v100, fine, sph_end_eff;            
            
            test_load := FCLoadId();           
            
            FCCalib test_load;      
            
            fc_cal := TRUE;
        
        ENDIF
                

        !Goto for if the socket closes
        start_no_cali:
       
        
        !Reset the trajectory pointers - essentially overwrite any old trajectories
        head := 1;
        tail := 1;    
        lst_trans_pnt := -1;
        lst_rot_pnt := -1;
        run_trajectory := FALSE;
        conc_count := 0;
        traj_done := FALSE;
        still_cnt := 0;
        queue_end := TRUE;
        
        !Open the local socket
        open_local_socket;

        TPWrite "Sockets open";
        
        accept_socket;

        ! Waiting for a connection request
        WHILE TRUE DO
            
            !if the real robot - check in bounds
            IF RobOS() THEN            
                in_bounds_check;            
            ENDIF
            
            !If the trajectory is finished, reset the state of the trajectory queue and associated variables
            IF traj_done = TRUE THEN
                head := 1;
                tail := 1;    
                lst_trans_pnt := -1;
                lst_rot_pnt := -1;
                run_trajectory := FALSE;
                conc_count := 0;
                still_cnt := 0;
                queue_end := TRUE;   
            ENDIF

            
            IF (DOutput(ROB_STATIONARY) = 1) THEN
                Incr still_cnt;
            ELSE
                still_cnt := 0;
            ENDIF
            
            !Check if the trajectory queue is finished
            IF (run_trajectory and (not queue_end)) and ((DOutput(move_started) = 1) and (DOutput(ROB_STATIONARY) = 1)) THEN
                TpWrite "Load Next Point - Force Mode: " + ValToStr(fc_mode);
                
                !Check if it needs to be a force mode trajectory or not 
                IF NOT fc_mode THEN
                    next_traj_pnt;
                ELSE
                    next_force_pnt;                    
                ENDIF
                
            ELSEIF (queue_end AND still_cnt > 20) THEN
                traj_done := TRUE;
                
            ENDIF
            
            !Recieve the string
            SocketReceive client_socket\Str:=receive_string;
            !Process the command
            cmd_handler receive_string;

        ENDWHILE
    
        ERROR
            !If the socket is closed remotely - send program pointer back to main - behaviour becomes undefined but may be able to retain some semblance of control
           IF ERRNO = ERR_SOCK_CLOSED THEN
           
                !! really feels like cheating! and itll just go down and down the main loop but...
                close_sockets;
                main;
               
                !GOTO start_no_cali;
           
           ENDIF
    


  

        
    !UNDO
    !Just incase something breaks - panic and close the sockets
    !close_sockets;
    

    ENDPROC
    
    !Testing function only
    PROC go_home()
         !Get the TCPs current position
        VAR robtarget curr_pos;
        VAR robtarget rob_home_pos; 
        
        curr_pos :=  CRobT();
        rob_home_pos := [[220.0, 1355.0, 955.0], curr_pos.rot, curr_pos.robconf, [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
        
        MoveL rob_home_pos, des_speed, fine, tool0;
    ENDPROC
    
    
    !Convenience wrapper
    PROC resp(string msg)        
        SocketSend client_socket\Str:= msg + "!";       
    ENDPROC


    !Procedure to open the socket on the controller
    PROC open_local_socket()
        
        SocketCreate server_socket;
        
        !Check if virutal controller or real
        !ROBOS =True is real controller
        IF ROBOS() THEN        
            SocketBind server_socket,"192.168.125.1", 8888;       
        
        ELSE
            SocketBind server_socket,"127.0.0.1", 8888;
        
        
        ENDIF
        
            SocketListen server_socket;

    ENDPROC

    !Accepts a socket connection
    PROC accept_socket()
        SocketAccept server_socket,client_socket\ClientAddress:=client_ip\Time:=WAIT_MAX;
    ENDPROC


    !Checks the position of the robot TCP and ensures that it is within acceptable/safe boundaries
    PROC in_bounds_check()
       
        
        !Set the bounds
        CONST num MAX_X := 800;
        CONST num MIN_X := -425;
        
        CONST num MAX_Y := 2650;
        CONST num MIN_Y := 1300;
        
        CONST num MAX_Z := 2000;
        CONST num MIN_Z := 95;
        

        
        !Get the TCPs current position
        VAR robtarget curr_pos;
        VAR robtarget rob_home_pos; 
        
        curr_pos :=  CRobT();
        rob_home_pos := [[220.0, 1355.0, 955.0], curr_pos.rot, curr_pos.robconf, [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
        
        !Compare the posiiton with the bounds
        IF curr_pos.trans.x >= MAX_X OR curr_pos.trans.x <= MIN_X OR curr_pos.trans.y >= MAX_Y OR curr_pos.trans.y <= MIN_Y OR curr_pos.trans.Z >= MAX_Z OR curr_pos.trans.Z <= MIN_Z THEN            
                    
            
            
            des_speed := [50, 500, 5000, 1000];
            
            
            !If it breaches any of the bound rules - emergency stop and stop the program
            MoveL rob_home_pos, des_speed, fine, tool0;
            StopMove \Quick; 
            
            ErrWrite "POS BOUND BREACH", "Outside of the acceptable bounds -moving home";
            
            
            
            Stop \NoRegain;
            
            
        ENDIF
        

        
        
    ENDPROC
    
    
    !Handles Commands
    PROC cmd_handler(string cmd)

        VAR string cmd_ID;
        VAR string cmd_req;
        VAR num req_len;

        !Strips the first four characters to identify the command
        cmd_ID:=StrPart(cmd,1,4);
        !Accesses the request related to the command
        req_len:=StrLen(cmd)-5;
        cmd_req:=StrPart(cmd,6,req_len);

        !TPWrite "ID:"+cmd_ID;
        !TPWrite "Request: "+cmd_req;

        !Match case the currently programmed commands
        TEST cmd_ID

 
        CASE "ECHO":
            !Repeats the string back to the controller
            resp("ECHO_MSG: "+cmd_req);

            !Close the sockets (should fix opening multiple similar sockets)
        CASE "CLOS":
            resp("CLOSING PORT");
            close_sockets;

            !Request the robot move to a specific position
        CASE "MVTO":
            move_to cmd_req;
            
        !Set the joints specifically        
        CASE "STJT":
            set_jnt cmd_req;
            
        !Set the orientation of the TCP
        CASE "STOR":
            set_ori cmd_req;
            
        !Set the speed of the robot
        CASE "STSP":
            set_speed cmd_req;
        
        !Move the tool relative to its current position
        CASE "MVTL":
            move_tool cmd_req;
        
        !Report the robots current position
        CASE "GTPS":
            report_pos;
            
        !Report the robots current orientation
        CASE "GTOR":
            report_orientation;
            
        !Report the robots current force
        CASE "GTFC":
            report_force;
            
        !Report the robots model number
        CASE "RMDL":
            report_model;
            
        !Report the robots joint angles (without external)
        CASE "GTJA":
            report_joints;
            
        !Report the moving state
        CASE "MVST":       
            Resp(ValToStr(DOutput(ROB_STATIONARY)));
            
        !Add a tranlation to the trajectory queue
        CASE "TQAD":
            traj_add_pnt cmd_req;
        
        !Add a rotation to the trajectory queue
        CASE "RQAD":
            traj_add_rot cmd_req;
            
            
        !Start the trajectory queue
        CASE "TJGO":
            run_trajectory := TRUE;
            resp("GOING");            
            SetDo move_started, 1;

            
        !Stop the trajectory queue
        CASE "TJST":
            run_trajectory := FALSE;
            resp("STOPPING");
            
        !Reports whether the trajectory queue has reached the end
        CASE "TJDN":        
            resp(ValToStr(traj_done));
            
        
        !Reports the Torque of every joint
        CASE "GTTQ":
            report_torque;
            
        !Sets the force mode
        CASE "STFM":
            set_force_mode cmd_req;
            
        !Sets the force config
        CASE "STFC":
            set_force_cntrl_config cmd_req;
        
        !Add a relative movement to the rel_move queue
        CASE "RLAD":
            force_add_pnt cmd_req;
            
        !Set the compensation movement for achieving the desired force
        CASE "FCCM":
            set_force_compensation cmd_req;
            
        !Attempt to find vertical force
        CASE "RQVF":
            find_vert_force;
        
        CASE "GTVF":
            resp(ValToStr(vert_force_found));        
        
        !if unprogrammed/unknown command is sent    
        DEFAULT:
            TPWrite "INVALID CMD: " + cmd_ID;
            resp("UNKNOWN CMD");

        ENDTEST

    ENDPROC

    
    
    
    !Add a point to the trajectory
    PROC traj_add_pnt(string add_traj_pos)
        
    
        !decode the target pos into a robtarget variable
        VAR robtarget rob_trgt_pos;
        VAR bool ok;
        !Get the current target
        VAR robtarget curr_trgt;
        
        IF(lst_rot_pnt = -1) THEN
            curr_trgt := CRobT(\Tool:=sph_end_eff \WObj:=wobj0);
            
        ELSE
            curr_trgt := traj_coord_queue{lst_rot_pnt};
            
        ENDIF
        
        !TpWrite ValToStr(add_traj_pos);
        
        !Should be able to convert to the robot target directly
        ok:= StrToVal(add_traj_pos ,rob_trgt_pos.trans);
        
        !Keep everything else the same
        !NOTE: NEED TO UPDATE SO Z ANGLE IS ALWAYS 0
        rob_trgt_pos.rot := curr_trgt.rot;        
        rob_trgt_pos.extax := curr_trgt.extax;        
        rob_trgt_pos.robconf := curr_trgt.robconf;
        
  
        !TPWrite("NEW: " + ValToStr(rob_trgt_pos.trans) + " " + ValToStr(rob_trgt_pos.rot));
        
        !If conversion okay
        IF(ok) THEN
            !add to the end of the queue
            traj_coord_queue{tail} := rob_trgt_pos;
            
            lst_trans_pnt := tail;
            
            !Increment the tail
            Incr tail;
            
            !Change the queue end flag
            queue_end := FALSE;
            
            !Change the trajectory done flag
            traj_done := FALSE;
            
            
            resp("OK");

        ENDIF
        
        IF NOT ok THEN
            !If something breaks
            TPWrite "Invalid target position";
            resp("INVALID POS");
        ENDIF       
        
        
    ENDPROC
    
    !Adds a relative xyz movement to the relative movement queue
    PROC force_add_pnt(string add_rel_mov)
        
       
        
        !Turn the string into a xyz coordinate
        VAR num new_pos{3};
        VAR bool ok;    
           
                
        !Convert XYZ strings into the coordinates
        ok := StrToVal(add_rel_mov, new_pos);        
    
        
        !If the conversion to num was okay 
        IF ok THEN
    
            !Add the coordinate to the queue
            rel_move_queue{tail, 1} := new_pos{1};
            rel_move_queue{tail, 2} := new_pos{2};
            rel_move_queue{tail, 3} := new_pos{3};
            
            
            !Increment the tail and set the flags
            Incr tail;
            
            queue_end := FALSE;
            traj_done := FALSE;
            
            !TpWrite "SENDING OK!";
                        
            resp("OK");
                     
            
        ENDIF
        
        IF NOT ok THEN
            !If something breaks
            TPWrite "Invalid relative movement";
            resp("INVALID RELMOV XYZ");
        ENDIF  
            
        
        
    ENDPROC
    
    
     !Add a point to the trajectory
    PROC traj_add_rot(string add_traj_ori)
        
    
        !decode the target pos into a robtarget variable
        VAR robtarget rob_trgt_ori;
        VAR orient targ_ori;
        VAR bool ok;
        !Get the most recent target from the queue
        VAR robtarget curr_trgt;
        IF(lst_trans_pnt = -1) THEN
            curr_trgt := CRobT(\Tool:=sph_end_eff \WObj:=wobj0);
            
        ELSE
            curr_trgt := traj_coord_queue{lst_trans_pnt};
            
        ENDIF
        
        !Should be able to convert to the robot target directly
        ok:= StrToVal(add_traj_ori, rob_trgt_ori.rot);
        
        !Keep everything else the same
        !NOTE: NEED TO UPDATE SO Z ANGLE IS ALWAYS 0
        rob_trgt_ori.trans := curr_trgt.trans;        
        rob_trgt_ori.extax := curr_trgt.extax;        
        rob_trgt_ori.robconf := curr_trgt.robconf;
        
  

        !If conversion okay
        IF(ok) THEN
            !add to the end of the queue
            traj_coord_queue{tail} := rob_trgt_ori;
            
            lst_rot_pnt := tail;
            
            !Increment the tail
            Incr tail;
            
            !Change the queue end flag
            queue_end := FALSE;
            
            !Change the trajectory done flag
            traj_done := FALSE;
            
            resp("OK");

        ENDIF
        
        IF NOT ok THEN
            !If something breaks
            TPWrite "Invalid target position";
            resp("INVALID ORI");
        ENDIF  
        
        
        
    ENDPROC
    
    
    !Move to the next point in the trajectory    
    PROC next_traj_pnt()          
        
        !Setup the trigger for the trajectory point
        VAR triggdata set_move_flag;
        
        !Access the first in the trajectory queue
        VAR robtarget next_trg;
        next_trg := traj_coord_queue{head};
        
        !Reset the start_move flag
        SetDO move_started, 0;
        
        !TPWrite "Signal Low";
        
              
        !Setup the trigger
        TriggIO set_move_flag, 0,\Dop:= move_started, 1;
        
        
        IF (conc_count < 5) THEN
            !Set the robot to move to the desired point
            TriggL \Conc, next_trg, des_speed, set_move_flag, fine , sph_end_eff;
            Incr conc_count;            
            
        ELSE
                    
            !Set the robot to move to the desired point
            TriggL next_trg, des_speed, set_move_flag, fine, sph_end_eff;
            
            conc_count := 0;            
        
        ENDIF
        
        !increment the head counter
        Incr head;
        
        !Check if the trajectory queue is empty and update the flags
        IF(head = tail) THEN
            queue_end := TRUE;
            run_trajectory := FALSE;
        ENDIF

    ENDPROC
    
    
    !Uses the relative movement queue combined iwth the force requirements to determine the movement
    PROC next_force_pnt()
        
        !Setup the trigger
        VAR triggdata set_move_flag;
        
        VAR num force_diff;
        
        !Access the xyz coords to move by    
        VAR num rel_move{3};
        rel_move{1} := rel_move_queue{head, 1};
        rel_move{2} := rel_move_queue{head, 2};
        rel_move{3} := rel_move_queue{head, 3};
        
        !Reset the start_move flag
        SetDO move_started, 0;        
              
        !Setup the trigger
        TriggIO set_move_flag, 0,\Dop:= move_started, 1; 
        
        
        
        
        !Calculate the difference between the desired force and the actual force 
        !FOR NOW ONLY Z
        IF NOT force_ax = "Z" THEN
            !PANIC!
            EXIT;
        ENDIF
       
        
        !Check to see if the movement should compensate
        IF fm_compensate THEN
        
            IF force_ax = "Z" THEN
                rel_move{3} := rel_move{3} + comp_dist;
            ENDIF
            
            !Turn off the compensation
            fm_compensate := FALSE;
            
        ENDIF
         
        
        !Check that the concurrent movements haven't been reached
        IF (conc_count < 5) THEN
            
            !Move the tool by the modified amount
            TriggL \Conc, RelTool( CRobT(\Tool:=sph_end_eff \WObj:=wobj0), rel_move{1}, rel_move{2}, rel_move{3}), des_speed, set_move_flag, fine , sph_end_eff;
            Incr conc_count;            
            
        ELSE    
            !Set the robot to move to the desired point
             TriggL RelTool( CRobT(\Tool:=sph_end_eff \WObj:=wobj0), rel_move{1}, rel_move{2}, rel_move{3}), des_speed, set_move_flag, fine , sph_end_eff;
            
             !Reset the concurrent move count
            conc_count := 0;            
        
        ENDIF
        
        
        
        !increment the head counter
        Incr head;
        
        !Check if the trajectory queue is empty and update the flags
        IF(head = tail) THEN
            queue_end := TRUE;
            run_trajectory := FALSE;
        ENDIF
        
        
          
        
    ENDPROC
    

    
  
    

    !Moves the robot end-affector to a specified posiiton via robtarget
    PROC move_to(string target_pos)
        !decode the target pos into a robtarget variable
        VAR robtarget rob_trgt_pos;
        VAR bool ok;
        !Get the current target
        VAR robtarget curr_trgt;
        curr_trgt := CRobT(\Tool:=sph_end_eff \WObj:=wobj0);        
        
        !Should be able to convert to the robot target directly
        ok:= StrToVal(target_pos,rob_trgt_pos.trans);
        
        !Keep everything else the same
        !NOTE: NEED TO UPDATE SO Z ANGLE IS ALWAYS 0
        rob_trgt_pos.rot := curr_trgt.rot;        
        rob_trgt_pos.extax := curr_trgt.extax;        
        rob_trgt_pos.robconf := curr_trgt.robconf;
        
        !TPWrite("CURR: " + ValToStr(curr_trgt.trans) + " " + ValToStr(curr_trgt.rot));
        !TPWrite("NEW: " + ValToStr(rob_trgt_pos.trans) + " " + ValToStr(rob_trgt_pos.rot));


        IF ok THEN                   
            !Move the robot to the target
            MoveJ rob_trgt_pos, des_speed, fine, sph_end_eff;

            resp("MVTO OK");
            
        ENDIF

        IF NOT ok THEN
            !If something breaks
            TPWrite "Invalid target position";
            resp("INVALID POS");
        ENDIF
    ENDPROC
    
     PROC set_ori(string ori)

        !setup the required variables

        VAR robtarget rob_trgt;
        VAR bool ok;

        !get the current target
        VAR robtarget curr_trgt;
        curr_trgt := CRobT(\Tool:=tool0 \WObj:=wobj0); 
        
        !Should be able to convert to the robot target directly
        ok:= StrToVal(ori,rob_trgt.rot);


        !keep everything the same
        rob_trgt.trans := curr_trgt.trans;
        rob_trgt.robconf := curr_trgt.robconf;
        rob_trgt.extax := curr_trgt.extax;

        !if the orientation req is okay - move the robots
        IF ok THEN                   
            !Move the robot to the target
            MoveJ rob_trgt, v100, fine, tool0;
            resp("STOR OK");
        ENDIF

        IF NOT ok THEN
            !If something breaks
            TPWrite "Invalid target ori";
            resp("INVALID ORI");
        ENDIF
    ENDPROC
    
    !Moves the robot to a set position via target joint angles
    PROC set_jnt(string target_jnts)
        !Declare the joint target
        VAR jointtarget jnt_trgt;
        VAR bool ok;
        
        TpWrite(ValToStr(target_jnts));

        
        !Convert the string into the joint targets
        ok := StrToVal(target_jnts, jnt_trgt);

        
        IF ok THEN
            
            TPWrite ValToStr(jnt_trgt);
            
            MoveAbsJ jnt_trgt, des_speed, fine, sph_end_eff;
            
            !Let the client know the move happened
            resp("STJT CMPL");
        
        ELSE
            !If something breaks
            TPWrite "Invalid target joints";
            resp("INVALID ANGLES");
            
        ENDIF

    ENDPROC
    
    PROC set_speed(string speed)
        
        VAR bool ok;
        
        !Convert the speed to the desired value
        ok := StrtoVal(speed, des_speed_num);
        des_speed := [des_speed_num, 500, 5000, 1000];
        
        resp("SPEED CHANGED");
        
    ENDPROC
    
    !move the tool relative to its current position
    PROC move_tool(string dists)
        
        !relative movements in the XYZ directions
        VAR num dX;
        VAR num dY;
        VAR num dZ;
        
        VAR bool ok;        
        
        
        VAR num start_char := 1;
        VAR num end_char;
        VAR num i := 0;
        VAR string curr_num;
        
        !Arrays in rapid start at 1 
        WHILE i < 3 DO                 
            
            
            !Update the end - find the comma
            end_char := StrFind(dists, start_char + 1, ",");  
                        
            IF i = 0 OR i = 1 THEN
            !Find the next value
            curr_num := StrPart(dists, start_char + 1, end_char - start_char - 1);     
            ENDIF
            
            !sort the variables
            IF i = 0 THEN    
               !TpWrite("X: " + curr_num);
               ok := StrToVal(curr_num, dX);   
            ELSEIF i = 1 THEN       
                !TpWrite("Y: " + curr_num);
                ok := StrToVal(curr_num, dY);
            ELSEIF i = 2 THEN
                !Special formatting to ignore square bracket
                curr_num := StrPart(dists, start_char + 1, StrLen(dists) - start_char - 1);   
                !TpWrite("Z: " + curr_num);
                ok := StrToVal(curr_num, dZ);
            ENDIF
            
            i := i + 1;
            
            !Move the starting point
            start_char := end_char;
        
        ENDWHILE        
        
        
        !Move the tool as described
        !MoveLSync RelTool( CRobT(\Tool:=tool1 \WObj:=wobj0), dX, dY, dZ , \Rx:= 0, \Ry:= 0. \Rz:= 0), v100, fine, tool1, "report_pos_and_force";
        
        IF conc_count < 5 THEN
            MoveL \conc, RelTool (CRobT(\Tool:=sph_end_eff \WObj:=wobj0), dX, dY, dZ), des_speed, fine, sph_end_eff;
            Incr conc_count;        
        ELSE
            MoveL RelTool (CRobT(\Tool:=sph_end_eff \WObj:=wobj0), dX, dY, dZ), des_speed, fine, sph_end_eff;
            conc_count := 0;
        ENDIF
        

        resp("MVTL CMPL");
        
    ENDPROC
    
    !Sends the current position to the tcp socket
    PROC report_pos()        
        resp(ValToStr(CPos(\Tool:=sph_end_eff \WObj:=wobj0)));  
    ENDPROC
    
    !Sends the current orientation to the tcp socket
    PROC report_orientation()
        
        VAR robtarget curr_targ;
        
        curr_targ := CRobT(\Tool:=sph_end_eff \WObj:=wobj0);
        
        !Send all of the euler angles
        resp("[" + ValToStr(EulerZYX(\X, curr_targ.rot)) + "," + ValToStr(EulerZYX(\Y, curr_targ.rot)) + "," + ValToStr(EulerZYX(\Z, curr_targ.rot)) + "]");
        
    
        
    
    ENDPROC
    
    !Report the force being enacted on the robot
    PROC report_force()
    
        !Check if the robot has actual forces to measure
        IF ROBOS() THEN        
            force_vector := FCGetForce(\Tool:=sph_end_eff \ContactForce);
            resp(ValToStr(force_vector));
            
           
            
        !Otherwise send a set of default values
        ELSE
            
            resp("{-1,-1,-1,-1,-1,-1}");
        
        ENDIF
        
        
        
    ENDPROC
    
    
    !Sends the current position and force to the tcp socket 
    PROC report_pos_and_force()
        
        resp(ValToStr(CPos(\Tool:=sph_end_eff \WObj:=wobj0)));
        
        force_vector := FCGetForce(\Tool:=sph_end_eff \ContactForce);
        
        resp(ValToStr(force_vector));
        
    
    ENDPROC
    
    !Report the robots model
    PROC report_model()
        resp(GetSysInfo(\RobotType));        
    ENDPROC
    
    !Report the robots joint angles
    PROC report_joints()
        !Get the joint angles
        VAR jointtarget joints;  
        
        joints := CJointT();
        
        resp(ValToStr(joints.robax));
        
    ENDPROC
    
    
    !Reports the robots measured torques
    PROC report_torque()
        
        !TpWrite ValToStr(GetMotorTorque(1));
        
        !Send the torques of each joint in a comma seperated format
        resp("{" + ValToStr(GetMotorTorque(1)) + "," 
                                            + ValToStr(GetMotorTorque(2)) + ","
                                            + ValToStr(GetMotorTorque(3)) + ","
                                            + ValToStr(GetMotorTorque(4)) + ","
                                            + ValToStr(GetMotorTorque(5)) + ","
                                            + ValToStr(GetMotorTorque(6)) + "}");
        
        
    ENDPROC

    !Procedure to close the sockets
    PROC close_sockets()
        !Close the sockets - inform the console
        SocketClose server_socket;
        SocketClose client_socket;
        TPWrite "Sockets Closed";
        !Exit the program
        EXIT;

    ENDPROC
    
    
    !Turn force control mode on/off
    PROC set_force_mode(string mode)
        
       !Turn the mode string to a bool
        VAR bool mode_setting;
        VAR bool ok;
        ok := StrToVal(mode, mode_setting);       
        !Check that it was converted okay
        IF NOT ok THEN           
            resp("FAILED TO SWAP");
            RETURN;
        ENDIF             
        
        !Use the bool to set the fc setting
        fc_mode := mode_setting;
        !Send the confirmation message
        resp("FM:"+ ValToStr(mode_setting));
    ENDPROC
    
    !Set the force control config
    PROC set_force_cntrl_config(string config)
        
        !Define all variables
        VAR string ax;
        VAR string targ_string;
        VAR num targ;
       
        VAR bool ok;
        
        !Find the dot in the string
        VAR num sep_pos;
        sep_pos := StrFind(config, 1, ".");
        
        !Split the axes and the target
        ax := StrPart(config, 1, sep_pos - 1);    
        targ_string := StrPart(config, sep_pos+1, StrLen(config) - sep_pos);
        
        !TpWrite targ_string;
        
        !Check that the specified axis is valid
        TEST ax
            CASE "X":
                force_ax := ax;
            CASE "Z":
                force_ax := ax;
            CASE "Y":
                force_ax := ax;
            DEFAULT:
                resp("INVALID AXIS CONFIG");
                RETURN;            
        ENDTEST
        
        !Cast target value to correct type
        ok := StrToVal(targ_string, targ);       
        
        !Check casting worked
        IF NOT ok THEN
            resp("FAILED TO SET CONFIG");
            RETURN;
        ENDIF        
         !Set the variables
        force_target := targ;
        
        !Send back confirmation message
        resp("FC:" + force_ax + "." + ValToStr(force_target));
        
        
    ENDPROC
    
    
    !Set the force compensation flag and the value
    PROC set_force_compensation(string comp_val)
        
        !Parse the value into the value variable
        Var bool ok;
        ok := StrToVal(comp_val, comp_dist);
        
        IF ok THEN            
            !set the compensation flag high
            fm_compensate := TRUE;            
            resp("OK");            
        ELSE
            resp("FAILED TO CONVERT");        
        ENDIF   
     
    ENDPROC
    
    
    !Move down until a vertical force is found
    PROC find_vert_force()
        
        !Distance robot can move in one move
        VAR num max_move := 0.1;
        VAR robtarget curr_pos;
        VAR fcforcevector curr_force_vec;
        
    
        in_bounds_check;
        
        !Reset the vert_force_found flag
        vert_force_found := FALSE;       
       
        resp("going");
        
        !Move down and check the current force compared to the desired force 
        WHILE(NOT vert_force_found) DO
            
            !Get current robot position
            curr_pos := CRobT(\Tool:=sph_end_eff \WObj:=wobj0);
            
            !Do a relative move downwards based on the maximum movement
            IF conc_count < 5 THEN
                MoveL \conc, RelTool( CRobT(\Tool:=sph_end_eff \WObj:=wobj0), 0, 0, -max_move), des_speed, fine, sph_end_eff;
                Incr conc_count;
            ELSE
                MoveL RelTool( CRobT(\Tool:=sph_end_eff \WObj:=wobj0), 0, 0, -max_move), des_speed, fine, sph_end_eff;
                conc_count := 0;
            ENDIF            
            
            curr_force_vec := FcGetForce(\Tool:=sph_end_eff, \ContactForce);
            
            !Check the if the vertical force has reached the desired target
            IF  curr_force_vec.zforce >= force_target THEN                
                !If so flick the vertical force found high
                vert_force_found := TRUE;
            ENDIF
            
            
            
            !Do a set of cmd handlers so that the computer can request data
            !FORCE_FOUND
            SocketReceive client_socket\Str:=receive_string;    
            cmd_handler receive_string;
            !POS
            SocketReceive client_socket\Str:=receive_string;    
            cmd_handler receive_string;
            !ORI
            SocketReceive client_socket\Str:=receive_string;    
            cmd_handler receive_string;
            !FORCE
            SocketReceive client_socket\Str:=receive_string;    
            cmd_handler receive_string;
            !ROB_MOV_STATE
            SocketReceive client_socket\Str:=receive_string;    
            cmd_handler receive_string;
            !TRAJ_DONE
            SocketReceive client_socket\Str:=receive_string;    
            cmd_handler receive_string;
            
                
        
        
                    
        ENDWHILE
        
        
        
        
    ENDPROC
    
    

ENDMODULE
