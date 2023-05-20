trj_file = csvread("zk_sdf.csv");
trj_file_T = trj_file';
csvwrite("zk_sdf.csv", trj_file_T);