MODULE tcp

    
    
    !Variables to store socket info and socket commands
    VAR socketdev server_socket;
    VAR socketdev client_socket;
    VAR string receive_string;
    VAR string client_ip;
    
    
     !Tool that has its base frame rotated to match the base frame orientation
     PERS tooldata tool1 := [TRUE, [[0, 0, 0], [0, 0.7071068, -0.7071068, 0]],
                        [0.97284, [0, 0, 0.001],[1, 0, 0, 0], 0, 0, 0]];
    
    
    VAR loaddata test_load :=[0.01,[0,0,20],[1,0,0,0],0,0,0];
    
    VAR fcforcevector test_force_vector;
    


            
        
     
    !Initialise Trajectory queue
    VAR robtarget traj_coord_queue{100000};    
    
    VAR num head := 1;
    VAR num tail := 1;
    
    !Go/Pause Flag
    VAR bool run_trajectory := FALSE;
    
    !traj done and still count
    VAR bool traj_done := FALSE;
    VAR num still_cnt := 0;
    
    
    !Empty Flag
    VAR bool queue_end := TRUE;    
    !Go msg workaround
    VAR bool go_msg := FALSE;
    
    


    PROC main()      

        
        
        
        
        !MoveL RelTool( CRobT(\Tool:=tool1 \WObj:=wobj0), 0, 0, 10), v100, fine, tool1;
               
        
        !Calibrate the load sensor - the documentation reccomends making a fine movement before the calibration   
        !test_load := FCLoadId();

        !MOVE HERE
        !FCCalib test_load;       
        
        
        
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
            
            
            
            !Check if the trajectory queue is finished - THE ISSUE LIES HERE - 
            IF (run_trajectory and (not queue_end)) and (still_cnt > 8 OR (NOT go_msg)) THEN
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
        RETRY;
        
        
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
            
        !Add to the trajectory queue
        CASE "TQAD":
            traj_add_pnt cmd_req;
            
        !Start the trajectory queue
        CASE "TJGO":
            run_trajectory := TRUE;
            go_msg := FALSE;

            
        !Stop the trajectory queue
        CASE "TJST":
            run_trajectory := FALSE;
            
        !Reports whether the trajectory queue has reached the end
        CASE "TJDN":        
            SocketSend client_socket\Str:= ValToStr(traj_done) + "!";
        
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
        curr_trgt := CRobT(\Tool:=tool1 \WObj:=wobj0);    
        
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
        
        !Access the first in the trajectory queue
        VAR robtarget next_trg;
        next_trg := traj_coord_queue{head};
        
        !TpWrite "GOTO: " + ValToStr(next_trg.trans);       
      
        
        !Set the robot to move to the desired point
        MoveL \Conc, next_trg, v5, fine, tool1;
        
        !increment the head counter
        Incr head;
        
        !Check if the trajectory queue is empty and update the flags
        IF(head = tail) THEN
            queue_end := TRUE;
            run_trajectory := FALSE;
        ENDIF
   
        
        IF(go_msg = FALSE) THEN
            SocketSend client_socket\Str:="GOING!";
            go_msg := TRUE;
        ENDIF

    ENDPROC
    

    !Moves the robot end-affector to a specified posiiton via robtarget
    PROC move_to(string target_pos)
        !decode the target pos into a robtarget variable
        VAR robtarget rob_trgt_pos;
        VAR bool ok;
        !Get the current target
        VAR robtarget curr_trgt;
        curr_trgt := CRobT(\Tool:=tool1 \WObj:=wobj0);        
        
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
            MoveJ rob_trgt_pos, v100, fine, tool1;

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
        
        

        !Convert the string into the joint targets
        ok := StrToVal(target_jnts, jnt_trgt);

        
        IF ok THEN
            
            TPWrite ValToStr(jnt_trgt);
            
            MoveAbsJ jnt_trgt, v100, fine, tool1;
            
            !Let the client know the move happened
            SocketSend client_socket\Str:= "STJT CMPL" + "!";
        
        ELSE
            !If something breaks
            TPWrite "Invalid target joints";
            SocketSend client_socket\Str:="INVALID ANGLES" + "!";
            
        ENDIF

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
        
        MoveL RelTool( CRobT(\Tool:=tool1 \WObj:=wobj0), dX, dY, dZ), v20, fine, tool1;
        
        
           
        !Get motor moving signal
        if DOutput(ROB_STATIONARY) <> 1 THEN 
            TPWrite "MOVING"; 
        endif
        

        SocketSend client_socket\Str:= "MVTL CMPL" + "!";
        
    ENDPROC
    
    !Sends the current position to the tcp socket
    PROC report_pos()        
        SocketSend client_socket\Str:= ValToStr(CPos(\Tool:=tool1 \WObj:=wobj0)) + "!";  
    ENDPROC
    
    !Sends the current orientation to the tcp socket
    PROC report_orientation()
        
        VAR robtarget curr_targ;
        
        curr_targ := CRobT(\Tool:=tool1 \WObj:=wobj0);
        
        !Send all of the euler angles
        SocketSend client_socket\Str:= "[" + ValToStr(EulerZYX(\X, curr_targ.rot)) + "," + ValToStr(EulerZYX(\Y, curr_targ.rot)) + "," + ValToStr(EulerZYX(\Z, curr_targ.rot)) + "]!";
        
    
        
    
    ENDPROC
    
    !Report the force being enacted on the robot
    PROC report_force()
    
        test_force_vector := FCGetForce(\Tool:=tool1);
        SocketSend client_socket\Str:= ValToStr(test_force_vector) + "!";
        
    ENDPROC
    
    
    !Sends the current position and force to the tcp socket 
    PROC report_pos_and_force()
        
        SocketSend client_socket\Str:= ValToStr(CPos(\Tool:=tool1 \WObj:=wobj0)) + "!";
        
        test_force_vector := FCGetForce(\Tool:=tool1);
        
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
