function json = read_json(fname)
   % Opens and reads the json file
   % lf           - Distance between front axle and center of mass
   % lr           - Distance between rear axle and center of mass
   % m            - Mass 
   % Izz          - Moment of Inertia
   % servo_offset - Servo offset 
   % rw           - Wheel radius
   % g            - Gravity constant
   % robot_frame  - Robot tracking frame eg. base_link
    fid = fopen(fname);
    raw = fread(fid,inf); 
    str = char(raw'); 
    fclose(fid); 
    json = jsondecode(str);
end