import os
from random import random
import numpy as np
params ={
			"generation_nb": 2000,
			"min_end_len": 0.10,
			"max_end_len": 0.18,
			"min_mid_len": 0.045,
			"max_mid_len": 0.125,
		}

def main(params):


	 src = open("modules/robot.xacro","r")
	 if src.mode == 'r':
		 contents =src.read()
		 print(contents)
	 src.close()

	 names=["end_leg_lenght_0", "end_leg_lenght_1", "end_leg_lenght_2", "end_leg_lenght_3", "end_leg_lenght_4", "end_leg_lenght_5", "mid_leg_lenght_0", "mid_leg_lenght_1", "mid_leg_lenght_2", "mid_leg_lenght_3", "mid_leg_lenght_4", "mid_leg_lenght_5"]
	 f = open('results/names.txt', "w+")
	 f.write("# names\n")        # column names
	 np.savetxt(f, np.array([names]).T,  fmt='%s')
	 f.close()

	 os.system('mkdir results')
	 for i in range(0, params["generation_nb"]):


		 lengths = []
		 # dest = open(len_file,"w+")
		 scaled_value = params["min_end_len"] + (random() * (params["max_end_len"] - params["min_end_len"]))
		 lengths.append(scaled_value)
		 contents_modif = contents.replace('<xacro:property name="end_leg_lenght_0" value="0.140"/>', '<xacro:property name="end_leg_lenght_0" value="'+str(scaled_value)+'"/>')
		 scaled_value = params["min_end_len"] + (random() * (params["max_end_len"] - params["min_end_len"]))
		 lengths.append(scaled_value)
		 contents_modif = contents_modif.replace('<xacro:property name="end_leg_lenght_1" value="0.140"/>', '<xacro:property name="end_leg_lenght_1" value="'+str(scaled_value)+'"/>')
		 scaled_value = params["min_end_len"] + (random() * (params["max_end_len"] - params["min_end_len"]))
		 lengths.append(scaled_value)
		 contents_modif = contents_modif.replace('<xacro:property name="end_leg_lenght_2" value="0.140"/>', '<xacro:property name="end_leg_lenght_2" value="'+str(scaled_value)+'"/>')
		 scaled_value = params["min_end_len"] + (random() * (params["max_end_len"] - params["min_end_len"]))
		 lengths.append(scaled_value)
		 contents_modif = contents_modif.replace('<xacro:property name="end_leg_lenght_3" value="0.140"/>', '<xacro:property name="end_leg_lenght_3" value="'+str(scaled_value)+'"/>')
		 scaled_value = params["min_end_len"] + (random() * (params["max_end_len"] - params["min_end_len"]))
		 lengths.append(scaled_value)
		 contents_modif = contents_modif.replace('<xacro:property name="end_leg_lenght_4" value="0.140"/>', '<xacro:property name="end_leg_lenght_4" value="'+str(scaled_value)+'"/>')
		 scaled_value = params["min_end_len"] + (random() * (params["max_end_len"] - params["min_end_len"]))
		 lengths.append(scaled_value)
		 contents_modif = contents_modif.replace('<xacro:property name="end_leg_lenght_5" value="0.140"/>', '<xacro:property name="end_leg_lenght_5" value="'+str(scaled_value)+'"/>')



		 scaled_value = params["min_mid_len"] + (random() * (params["max_mid_len"] - params["min_mid_len"]))
		 lengths.append(scaled_value)
		 contents_modif = contents_modif.replace('<xacro:property name="mid_leg_lenght_0" value="0.085"/>', '<xacro:property name="mid_leg_lenght_0" value="'+str(scaled_value)+'"/>')
		 scaled_value = params["min_mid_len"] + (random() * (params["max_mid_len"] - params["min_mid_len"]))
		 lengths.append(scaled_value)
		 contents_modif = contents_modif.replace('<xacro:property name="mid_leg_lenght_1" value="0.085"/>', '<xacro:property name="mid_leg_lenght_1" value="'+str(scaled_value)+'"/>')
		 scaled_value = params["min_mid_len"] + (random() * (params["max_mid_len"] - params["min_mid_len"]))
		 lengths.append(scaled_value)
		 contents_modif = contents_modif.replace('<xacro:property name="mid_leg_lenght_2" value="0.085"/>', '<xacro:property name="mid_leg_lenght_2" value="'+str(scaled_value)+'"/>')
		 scaled_value = params["min_mid_len"] + (random() * (params["max_mid_len"] - params["min_mid_len"]))
		 lengths.append(scaled_value)
		 contents_modif = contents_modif.replace('<xacro:property name="mid_leg_lenght_3" value="0.085"/>', '<xacro:property name="mid_leg_lenght_3" value="'+str(scaled_value)+'"/>')
		 scaled_value = params["min_mid_len"] + (random() * (params["max_mid_len"] - params["min_mid_len"]))
		 lengths.append(scaled_value)
		 contents_modif = contents_modif.replace('<xacro:property name="mid_leg_lenght_4" value="0.085"/>', '<xacro:property name="mid_leg_lenght_4" value="'+str(scaled_value)+'"/>')
		 scaled_value = params["min_mid_len"] + (random() * (params["max_mid_len"] - params["min_mid_len"]))
		 lengths.append(scaled_value)
		 contents_modif = contents_modif.replace('<xacro:property name="mid_leg_lenght_5" value="0.085"/>', '<xacro:property name="mid_leg_lenght_5" value="'+str(scaled_value)+'"/>')

		 print(lengths)
		 len_file = 'results/lengthes_' + str(i) +'.txt'
		 f2 = open(len_file, "w+")
 	  	 f2.write("# lengths\n")        # column names
 	  	 np.savetxt(f2, np.array([lengths]).T)
		 f2.close()


		 dest = open("modules/pexod.xacro","w+")
		 dest.write(contents_modif)
		 dest.close()

		 cmd = 'rosrun xacro xacro --inorder -o '
		 filename = 'results/pexod_' + str(i) +'.urdf'
		 cmd = cmd + filename +' pexod.xacro'
		 print(cmd)
		 os.system(cmd)


if __name__== "__main__":
  main(params)
  a = np.loadtxt('results/lengthes_0.txt', unpack=True)
  print(a)
