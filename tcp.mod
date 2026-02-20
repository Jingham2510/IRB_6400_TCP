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

    VAR num conc_count := 0;
    
    
    !ROBOT VARIABLES
    VAR num des_speed_num := 50;
    VAR speeddata des_speed := [50, 500, 5000, 1000];
    
    
    
    !Used during phase 1 of the geo loading
    VAR bool vert_force_found := FALSE;
    
    
    
    !EGM DATA
    VAR egmident egmID;
    VAR egmstate egm_state;
    VAR bool egm_running := FALSE;
    VAR bool egm_speed := FALSE;
    VAR bool egm_pose := FALSE;
    
    VAR pose posecor:= [[0,0,0],[1,0,0,0]];
    VAR pose posesense:=[[0,0,0],[1,0,0,0]];
    
    CONST egm_minmax egm_minmax_lin:=[-0.1,+0.1];
    CONST egm_minmax egm_minmax_rot:=[-0.1,+0.1];
    
    PERS Dnum egm_data_from_sensor{40};
    
    
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
        CONST num MAX_X := 650;
        CONST num MIN_X := -425;
        
        CONST num MAX_Y := 2650;
        CONST num MIN_Y := 1350;
        
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
            
        !Reports the Torque of every joint
        CASE "GTTQ":
            report_torque;
        !Connects the EGM server in cartesian pose mode
        CASE "EGPS":
            EGM_connect_pose;
            
        !Connects the EGM server in joint mode
        CASE "EGJT":
            EGM_connect_joint;            
            
        !Starts an EGM stream in speed (linear) control mode
        CASE "EGSS":
            EGM_start_stream_speed;
            
         !Starts an EGM stream in position control mode
        CASE "EGST":
            EGM_start_stream;
            
            
        !Stops the EGM stream    
        CASE "EGSP":
            EGM_stop_stream;
            
        !if unprogrammed/unknown command is sent    
        DEFAULT:
            TPWrite "INVALID CMD: " + cmd_ID;
            resp("UNKNOWN CMD");

        ENDTEST

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
        curr_trgt := CRobT(\Tool:=sph_end_eff \WObj:=wobj0); 
        
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

        !TpWrite(ValToStr(target_jnts));


        !Convert the string into the joint targets
        ok:=StrToVal(target_jnts,jnt_trgt.robax);


        IF ok THEN


            TPWrite ValToStr(jnt_trgt);

            IF conc_count<5 THEN

                MoveAbsJ\Conc,jnt_trgt,des_speed \T:=0.001 ,fine, sph_end_eff;

                conc_count:=conc_count+1;

            ELSE
                MoveAbsJ\Conc,jnt_trgt,des_speed \T:=0.001,fine,sph_end_eff;

            ENDIF


            !Let the client know the move happened
            !resp("STJT CMPL");

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
        !MoveLSync RelTool( CRobT(\Tool:=sph_end_eff \WObj:=wobj0), dX, dY, dZ , \Rx:= 0, \Ry:= 0. \Rz:= 0), v100, fine, sph_end_eff, "report_pos_and_force";
        
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
    
    
    
    
    
    !Connects to the EGM host server  setup for xyz cartesian control
    PROC EGM_connect_pose() 
        
        resp("opening UDP");
        
        !Reset the EGM ID just incase
        EgmReset egmID;
        
        !Get the ID to control the EGM connection
        EGMGetId egmID;
        
        IF RobOs() THEN
            
            EGMSetupUC ROB_1, egmID, "default", "RemoteUcDev", \Pose, \CommTimeout:=100000;     
        ELSE
            
             EGMSetupUC ROB_1, egmID, "default", "LocalUcDev", \Pose, \CommTimeout:=100000;     
        ENDIF        
          
        
        TPWrite "UDP (pose mode) Connected!";   
        
    ENDPROC
    
    
    
    !Connects to the EGM host server  setup for xyz cartesian control
    PROC EGM_connect_joint() 
        
        resp("opening UDP");
        
        !Reset the EGM ID just incase
        EgmReset egmID;
        
        !Get the ID to control the EGM connection
        EGMGetId egmID;
        
        IF RobOs() THEN
            
            EGMSetupUC ROB_1, egmID, "default", "RemoteUcDev", \Joint;     
        ELSE
            
             EGMSetupUC ROB_1, egmID, "default", "LocalUcDev", \Joint;     
        ENDIF        
        
        !Enable the joint
        EGMActJoint egmID,\DataFromSensor:= egm_data_from_sensor, \MaxSpeedDeviation:= 50;
          
        egm_pose := FALSE;
        
        TPWrite "UDP (joint mode) Connected!";   
        
    ENDPROC
    
    
    
    PROC EGM_start_stream_speed()
        EGMStreamStart egmID;
        resp("Stream started");     
      
        egm_data_from_sensor{1} := 0;
        
        
        
        EGMActPose egmID, \Tool:= sph_end_eff, \DataFromSensor:= egm_data_from_sensor,
                        posecor,EGM_FRAME_BASE, posesense,EGM_FRAME_BASE,
                        \X:= egm_minmax_lin,\Y:= egm_minmax_lin, \Z:= egm_minmax_lin,
                        \rx:=egm_minmax_rot, \ry:=egm_minmax_rot, \rz:=egm_minmax_rot,
                        \MaxSpeedDeviation:= 50;
                        
        EGMRunPose egmID, EGM_STOP_HOLD \NoWaitCond, 
            \x, \y, \z,
            \PosCorrGain:= 0;            
            
        EGMWaitCond egmId;
        
        EGMStop egmId, EGM_STOP_HOLD;
        
    ENDPROC
    
    PROC EGM_start_stream()
        EGMStreamStart egmID;
        resp("Stream started");               
       
        egm_data_from_sensor{1} := 0;
           
        EGMActPose egmID, \Tool:= sph_end_eff, \DataFromSensor:= egm_data_from_sensor,
            posecor,EGM_FRAME_BASE, posesense,EGM_FRAME_BASE,
        \X:= egm_minmax_lin,\Y:= egm_minmax_lin, \Z:= egm_minmax_lin,
        \rx:=egm_minmax_rot, \ry:=egm_minmax_rot, \rz:=egm_minmax_rot,
        \MaxSpeedDeviation:= 50;
        
        EGMRunPose egmID, EGM_STOP_HOLD, 
            \x, \y, \z,
            \PosCorrGain:= 1;       
            
        EGMWaitCond egmId;
        
        EGMStop egmId, EGM_STOP_HOLD;
    ENDPROC
    
    PROC EGM_start_stream_joint()
        
         EGMStreamStart egmID;
        resp("Stream started");               
       
        egm_data_from_sensor{1} := 0;
           
       EGMActJoint egmID, \DataFromSensor:= egm_data_from_sensor, \MaxSpeedDeviation:= 50;
                    
        EGMRunJoint EGMid, EGM_STOP_HOLD,
            \J1, \J2, \J3, \J4, \J5, \J6 
            \PosCorrGain := 1;
            
        EGMWaitCond egmId;
        
        EGMStop egmId, EGM_STOP_HOLD;
        
    ENDPROC
    
       
    PROC EGM_start_stream_speed_joint()
        
         EGMStreamStart egmID;
        resp("Stream started");               
       
        egm_data_from_sensor{1} := 0;
           
       EGMActJoint egmID, \DataFromSensor:= egm_data_from_sensor, \MaxSpeedDeviation:= 50;
                    
        EGMRunJoint EGMid, EGM_STOP_HOLD,
            \J1, \J2, \J3, \J4, \J5, \J6 
            \PosCorrGain := 0;
            
        EGMWaitCond egmId;
        
        EGMStop egmId, EGM_STOP_HOLD;
        
    ENDPROC
    
    PROC EGM_stop_stream()
        EGMStreamStop egmID;
        resp("Stream stopped");
        
        egm_running := FALSE;
    ENDPROC
    

ENDMODULE
