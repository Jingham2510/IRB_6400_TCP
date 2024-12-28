MODULE tcp

    
    
    !Variables to store socket info and socket commands
    VAR socketdev server_socket;
    VAR socketdev client_socket;
    VAR string receive_string;
    VAR string client_ip;
    
    VAR loaddata test_load :=[0.001,[0,0,0.001],[1,0,0,0],0,0,0];
    
    VAR fcforcevector test_force_vector;

    PROC main()
        
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
            
            
            
        
        !if unprogrammed/unknown command is sent    
        DEFAULT:
            TPWrite "INVALID CMD: " + cmd_ID;
            SocketSend client_socket\Str:="UNKNOWN CMD" + "!";

        ENDTEST

    ENDPROC


    !Moves the robot end-affector to a specified posiiton via robtarget
    PROC move_to(string target_pos)
        !decode the target pos into a robtarget variable
        VAR robtarget rob_trgt_pos;

        VAR bool ok;
        
        !Should be able to convert to the robot target directly
        ok:= StrToVal(target_pos,rob_trgt_pos);

        !Write out the robot target just to check
        TPWrite ValToStr(rob_trgt_pos);

        
        IF ok THEN
                    
            !Move the robot to the target
            MoveJ rob_trgt_pos, v1000, z20, tool0;
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
            
            MoveAbsJ jnt_trgt, v100, fine, tool0;
            
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
        MoveLSync RelTool( CRobT(\Tool:=tool0 \WObj:=wobj0), dX, dY, dZ), v100, fine, tool0, "report_pos_and_force";
        
        !SocketSend client_socket\Str:= "MVTL CMPL";
        
    ENDPROC
    
    !Sends the current position to the tcp socket
    PROC report_pos()        
        SocketSend client_socket\Str:= ValToStr(CPos(\Tool:=tool0 \WObj:=wobj0)) + "!";  
    ENDPROC
    
    !Sends the current orientation to the tcp socket
    PROC report_orientation()
        
        !Bit buggy... doesnt actually report any sort of angle...
        
        !Send all of the euler angles
        SocketSend client_socket\Str:= ValToStr(EulerZYX(\Z, wobj0.oframe.rot)) + "," + ValToStr(EulerZYX(\Y, wobj0.oframe.rot)) + "," + ValToStr(EulerZYX(\X, wobj0.oframe.rot)) + "!";
        
    
        
    
    ENDPROC
    
    !Report the force being enacted on the robot
    PROC report_force()
    
        test_force_vector := FCGetForce(\Tool:=tool0);
        SocketSend client_socket\Str:= ValToStr(test_force_vector) + "!";
        
    ENDPROC
    
    
    !Sends the current position and force to the tcp socket 
    PROC report_pos_and_force()
        
        SocketSend client_socket\Str:= ValToStr(CPos(\Tool:=tool0 \WObj:=wobj0)) + "!";
        
        test_force_vector := FCGetForce(\Tool:=tool0);
        
        SocketSend client_socket\Str:= ValToStr(test_force_vector) + "!";
    
    ENDPROC
    
    !Report the robots model
    PROC report_model()
        SocketSend client_socket\Str:= GetSysInfo(\RobotType) + "!";        
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
