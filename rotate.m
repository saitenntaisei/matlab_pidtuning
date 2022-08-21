scaleInput = 1;
pcPythonExe='C:\Users\saiten\AppData\Local\Programs\Python\Python39\python.exe';
%[ver,exec,loaded]=pyversion(pcPythonExe);
%pyversion;
folderPath = fullfile(pwd,"2021_Clione2-develop");
ros2genmsg(folderPath);
folderPath =fullfile(pwd,"rotate"); %rosbag2/rosbag2_2022_05_18-17_46_59
bag = ros2bag(folderPath);
bag2info = ros2("bag","info",folderPath);
bagselimu = select(bag,"Time",[bag.StartTime, bag.EndTime],"Topic","/sensor/imu");
bagselcmd = select(bag,"Time",[bag.StartTime, bag.EndTime],"Topic","/command/reference");
msgsFilteredimu = readMessages(bagselimu);
msgsFilteredcmd = readMessages(bagselcmd);
ts_imu = timeseries(bagselimu);
ts_cmd= timeseries(bagselcmd);
imu=cell2mat(msgsFilteredimu);
cmd=cell2mat(msgsFilteredcmd);
imuangular= [imu.angular];
imuAngularZ=[imuangular.z];
cmdangular=[cmd.angular];
cmdAngularZ=[cmdangular.z];
t_imu=ts_imu.Data;
t_cmd=ts_cmd.Data;
taxis_imu=double(t_imu.MessageList.Time)./10^9;
taxis_cmd=double(t_cmd.MessageList.Time)./10^9;

velAngularZ = imuAngularZ;
[velAngularZ,taxis_imu] = resample(velAngularZ, taxis_imu,10);
[cmdAngularZ,taxis_cmd]= resample(cmdAngularZ,taxis_cmd,10);
figure;

plot(1:size(taxis_imu),velAngularZ);
hold on;
plot(1:size(taxis_cmd),cmdAngularZ.*scaleInput);
hold off;
figure;
dt=0.1;
len =min(size(taxis_imu),size(taxis_cmd));
z = iddata(transpose(velAngularZ(1:len)), transpose(cmdAngularZ(1:len)).*scaleInput, dt);
plot(z);
%input_dim = 1; % 入力のチャンネルを指定
%output_dim  = 1; % 出力のチャンネルを指定
%GS = spa(z(:,output_dim,input_dim));
%h = bodeplot(GS);
%pole_num = 1; % 極の数を指定
%zero_num = 0; % 零点の数を指定
%mtf = tfest(z(:,output_dim,input_dim), pole_num, zero_num);