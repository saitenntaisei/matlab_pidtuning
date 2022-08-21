scaleInput = 0.08;
pcPythonExe='C:\Users\saiten\AppData\Local\Programs\Python\Python39\python.exe';
%[ver,exec,loaded]=pyversion(pcPythonExe);
%pyversion;
folderPath = fullfile(pwd,"2021_Clione2-develop");
ros2genmsg(folderPath);
folderPath =fullfile(pwd,"lateral");
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
imulinear= [imu.linear];
imuLinearY=[imulinear.y];
cmdlinear=[cmd.linear];
cmdLinearY=[cmdlinear.y];
t_imu=ts_imu.Data;
t_cmd=ts_cmd.Data;
taxis_imu=double(t_imu.MessageList.Time)./10^9;
taxis_cmd=double(t_cmd.MessageList.Time)./10^9;

velLinearY = cumtrapz(double(taxis_imu),imuLinearY);
[velLinearY,taxis_imu] = resample(velLinearY, taxis_imu,10);
[cmdLinearY,taxis_cmd]= resample(cmdLinearY,taxis_cmd,10);
figure;

plot(1:size(taxis_imu),velLinearY);
hold on;
plot(1:size(taxis_cmd),cmdLinearY.*scaleInput);
hold off;
figure;
dt=0.1;
len =min(size(taxis_imu),size(taxis_cmd));
z = iddata(transpose(velLinearY(1:len)), transpose(cmdLinearY(1:len).*scaleInput), dt);
plot(z);
%input_dim = 1; % 入力のチャンネルを指定
%output_dim  = 1; % 出力のチャンネルを指定
%GS = spa(z(:,output_dim,input_dim));
%h = bodeplot(GS);
%pole_num = 1; % 極の数を指定
%zero_num = 0; % 零点の数を指定
%mtf = tfest(z(:,output_dim,input_dim), pole_num, zero_num);