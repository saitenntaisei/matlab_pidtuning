M = csvread("current_output1.csv");
cmdLinearX=M(:,2)
velLinearX=M(:,3);
figure;
dt=0.01;

len =min(size(cmdLinearX),size(velLinearX));
z = iddata(velLinearX(1:len), cmdLinearX(1:len), dt);
input_dim = 1; % 入力のチャンネルを指定
output_dim  = 1; % 出力のチャンネルを指定
GS = spa(z(:,output_dim,input_dim));
h = bodeplot(GS);
pole_num = 1; % 極の数を指定
zero_num = 0; % 零点の数を指定
mtf = tfest(z(:,output_dim,input_dim), pole_num, zero_num);
