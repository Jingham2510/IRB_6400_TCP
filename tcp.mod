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
    
    VAR fcforcevector test_force_vector;
    
    


    !If force control has been calibrated
    VAR bool fc_cal := FALSE;
     
    !Initialise Trajectory queue
    VAR robtarget traj_coord_queue{100000};    
    
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
    !Go msg workaround
    VAR bool go_msg := FALSE;
    
    !ROBOT VARIABLES
    VAR num des_speed_num := 50;
    VAR speeddata des_speed := [50, 500, 5000, 1000];


    PROC main()      
        



        SetDo move_started, 0;
        
        !Checks if force calibraiton is required or not
        !Not required for the virtual controller
        IF ROBOS() AND (NOT fc_cal) THEN                
     
            !Calibrate the load sensor - the documentation reccomends making a fine movement before the calibration   
            test_load := FCLoadId();
            
            MoveL RelTool( CRobT(\Tool:=sph_end_eff \WObj:=wobj0), 0, 0, -10), v100, fine, sph_end_eff;
    
            !MOVE HERE
            FCCalib test_load;      
            
            fc_cal := TRUE;
        
        ENDIF
                

        !Goto for if the socket closes
        start_no_cali:
        
        
        !Open the local socket
        open_local_socket;

        TPWrite "Sockets open";
        
        accept_socket;

        ! Waiting for a connection request
        WHILE TRUE DO
            

            
            IF (DOutput(ROB_STATIONARY) = 1) THEN
                Incr still_cnt;
            ELSE
                still_cnt := 0;
            ENDIF
            
            !Check if the trajectory queue is finished
            IF (run_trajectory and (not queue_end)) and ((DOutput(move_started) = 1) and (DOutput(ROB_STATIONARY) = 1)) THEN
                TpWrite "Load Next Point";
                next_traj_pnt;
                
            ELSEIF (queue_end AND still_cnt > 20) THEN
                traj_done := TRUE;
                
            ENDIF
            
            !Recieve the string
            SocketReceive client_socket\Str:=receive_string;
            !Process the command
            cmd_handler receive_string;

        ENDWHILE
    
        ERROR
            !If the socket is closed remotely - send pp back to main - behaviour becomes undefined but may be able to retain some semblance of control
           IF ERRNO = ERR_SOCK_CLOSED THEN
           
                !! really feels like cheating! and itll just go down and down the main loop but...
                close_sockets;
                main;
               
                !GOTO start_no_cali;
           
           ENDIF
    


  

        
    UNDO
        !Just incase something breaks - panic and close the sockets
        close_sockets;
    

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
            SocketSend client_socket\Str:="ECHO_MSG: "+cmd_req + "!";

            !Close the sockets (should fix opening multiple similar sockets)
        CASE "CLOS":
            SocketSend client_socket\Str:="CLOSING PORT" + "!";
            close_sockets;

            !Request the robot move to a specific position
        CASE "MVTO":
            move_to cmd_req;
            
        !Set the joints specifically        
        CASE "STJT":
            set_jnt cmd_req;
            
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
       
            SocketSend client_socket\Str:= ValToStr(DOutput(ROB_STATIONARY)) + "!";
            
        !Add a tranlation to the trajectory queue
        CASE "TQAD":
            traj_add_pnt cmd_req;
        
        !Add a rotation to the trajectory queue
        CASE "RQAD":
            traj_add_rot cmd_req;
            
            
        !Start the trajectory queue
        CASE "TJGO":
            run_trajectory := TRUE;
            go_msg := FALSE;
            SocketSend client_socket\Str:="GOING!";
            go_msg := TRUE;
            SetDo move_started, 1;

            
        !Stop the trajectory queue
        CASE "TJST":
            run_trajectory := FALSE;
            SocketSend client_socket\Str:="STOPPING!";
            
        !Reports whether the trajectory queue has reached the end
        CASE "TJDN":        
            SocketSend client_socket\Str:= ValToStr(traj_done) + "!";
            
        
        !Reports the Torque of every joint
        CASE "GTTQ":
            report_torque;
        
        
        !if unprogrammed/unknown command is sent    
        DEFAULT:
            TPWrite "INVALID CMD: " + cmd_ID;
            SocketSend client_socket\Str:="UNKNOWN CMD" + "!";

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
            
            
            SocketSend client_socket\Str:= "OK!";

        ENDIF
        
        IF NOT ok THEN
            !If something breaks
            TPWrite "Invalid target position";
            SocketSend client_socket\Str:="INVALID POS" + "!";
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
            
            SocketSend client_socket\Str:= "OK!";

        ENDIF
        
        IF NOT ok THEN
            !If something breaks
            TPWrite "Invalid target position";
            SocketSend client_socket\Str:="INVALID POS" + "!";
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
        
        TPWrite "Signal Low";
        
              
        !Setup the trigger
        TriggIO set_move_flag, 0,\Dop:= move_started, 1;
        
        IF (conc_count < 5) THEN
            
            !Set the robot to move to the desired point
            TriggL \Conc, next_trg, des_speed, set_move_flag, fine , sph_end_eff;
            
            Incr conc_count;
            
        ELSE
            !Set the robot to move to the desired point
            TriggL next_trg, des_speed, set_move_flag, fine , sph_end_eff;
            
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

            SocketSend client_socket\Str:="MVTO OK" + "!";
            
        ENDIF

        IF NOT ok THEN
            !If something breaks
            TPWrite "Invalid target position";
            SocketSend client_socket\Str:="INVALID POS" + "!";
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
            SocketSend client_socket\Str:= "STJT CMPL" + "!";
        
        ELSE
            !If something breaks
            TPWrite "Invalid target joints";
            SocketSend client_socket\Str:="INVALID ANGLES" + "!";
            
        ENDIF

    ENDPROC
    
    PROC set_speed(string speed)
        
        VAR bool ok;
        
        !Convert the speed to the desired value
        ok := StrtoVal(speed, des_speed_num);
        des_speed := [des_speed_num, 500, 5000, 1000];
        
        SocketSend client_socket\Str:= "SPEED CHANGED!";
        
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
               TpWrite("X: " + curr_num);
               ok := StrToVal(curr_num, dX);   
            ELSEIF i = 1 THEN       
                TpWrite("Y: " + curr_num);
                ok := StrToVal(curr_num, dY);
            ELSEIF i = 2 THEN
                !Special formatting to ignore square bracket
                curr_num := StrPart(dists, start_char + 1, StrLen(dists) - start_char - 1);   
                TpWrite("Z: " + curr_num);
                ok := StrToVal(curr_num, dZ);
            ENDIF
            
            i := i + 1;
            
            !Move the starting point
            start_char := end_char;
        
        ENDWHILE        
        
        
        !Move the tool as described
        !MoveLSync RelTool( CRobT(\Tool:=tool1 \WObj:=wobj0), dX, dY, dZ , \Rx:= 0, \Ry:= 0. \Rz:= 0), v100, fine, tool1, "report_pos_and_force";
        
        MoveL RelTool( CRobT(\Tool:=sph_end_eff \WObj:=wobj0), dX, dY, dZ), des_speed, fine, sph_end_eff;
        
        
           
        !Get motor moving signal
        if DOutput(ROB_STATIONARY) <> 1 THEN 
            TPWrite "MOVING"; 
        endif
        

        SocketSend client_socket\Str:= "MVTL CMPL" + "!";
        
    ENDPROC
    
    !Sends the current position to the tcp socket
    PROC report_pos()        
        SocketSend client_socket\Str:= ValToStr(CPos(\Tool:=sph_end_eff \WObj:=wobj0)) + "!";  
    ENDPROC
    
    !Sends the current orientation to the tcp socket
    PROC report_orientation()
        
        VAR robtarget curr_targ;
        
        curr_targ := CRobT(\Tool:=sph_end_eff \WObj:=wobj0);
        
        !Send all of the euler angles
        SocketSend client_socket\Str:= "[" + ValToStr(EulerZYX(\X, curr_targ.rot)) + "," + ValToStr(EulerZYX(\Y, curr_targ.rot)) + "," + ValToStr(EulerZYX(\Z, curr_targ.rot)) + "]!";
        
    
        
    
    ENDPROC
    
    !Report the force being enacted on the robot
    PROC report_force()
    
        IF ROBOS() THEN        
            test_force_vector := FCGetForce(\Tool:=sph_end_eff \ContactForce);
            SocketSend client_socket\Str:= ValToStr(test_force_vector) + "!";
            
        ELSE
            SocketSend client_socket\Str:= "{-1,-1,-1,-1,-1,-1}!";
        
        ENDIF
        
        
        
    ENDPROC
    
    
    !Sends the current position and force to the tcp socket 
    PROC report_pos_and_force()
        
        SocketSend client_socket\Str:= ValToStr(CPos(\Tool:=sph_end_eff \WObj:=wobj0)) + "!";
        
        test_force_vector := FCGetForce(\Tool:=sph_end_eff);
        
        SocketSend client_socket\Str:= ValToStr(test_force_vector) + "!";
        
    
    ENDPROC
    
    !Report the robots model
    PROC report_model()
        SocketSend client_socket\Str:= GetSysInfo(\RobotType) + "!";        
    ENDPROC
    
    !Report the robots joint angles
    PROC report_joints()
        !Get the joint angles
        VAR jointtarget joints;  
        
        joints := CJointT();
        
        SocketSend client_socket\Str:= ValToStr(joints.robax) + "!";
        
    ENDPROC
    
    
    !Reports the robots measured torques
    PROC report_torque()
        
        !TpWrite ValToStr(GetMotorTorque(1));
        
        !Send the torques of each joint in a comma seperated format
        SocketSend client_socket\Str:= "{" + ValToStr(GetMotorTorque(1)) + "," 
                                            + ValToStr(GetMotorTorque(2)) + ","
                                            + ValToStr(GetMotorTorque(3)) + ","
                                            + ValToStr(GetMotorTorque(4)) + ","
                                            + ValToStr(GetMotorTorque(5)) + ","
                                            + ValToStr(GetMotorTorque(6)) + "}!";
        
        
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

ENDMODULE
